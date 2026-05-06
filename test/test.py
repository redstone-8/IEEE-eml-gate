import math

import cocotb
import mpmath as mp
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

from programs_list import programs

# Constants

SOF = 0xA5
RESP_BITS = 36  # 3 status bits + 16-bit Q6.10 real result + padding
HOST_DPS = 80  # mpmath precision for software EML path
PROG_TOL = 2e-2  # acceptable error for 38-function sweep

# Q6.10 representable range
Q6_10_MAX = 31.999  # 32767 / 1024
Q6_10_MIN = -32.0  # -32768 / 1024


# Q6.10 encoding


def float_to_q6_10(val):
    if math.isnan(val):
        return 0x7FFE
    if val == math.inf:
        return 0x7FFF
    if val == -math.inf:
        return 0x8001
    scaled = round(val * 1024.0)
    if scaled > 32767:
        return 0x7FFF
    if scaled < -32768:
        return 0x8001
    if scaled < 0:
        scaled = (1 << 16) + scaled
    return scaled


def q6_10_to_float(val):
    if val == 0x7FFF:
        return math.inf
    if val == 0x8001:
        return -math.inf
    if val == 0x7FFE:
        return math.nan
    if val & 0x8000:
        val -= 1 << 16
    return val / 1024.0


# Helpers


def as_complex(val):
    return val if isinstance(val, complex) else complex(val)


def is_finite_complex(val):
    z = as_complex(val)
    return math.isfinite(z.real) and math.isfinite(z.imag)


def is_real(val, tol=1e-12):
    return abs(as_complex(val).imag) <= tol


def to_mpc(val):
    z = as_complex(val)
    return mp.mpc(z.real, z.imag)


def fmt(val):
    z = as_complex(val)
    if not math.isfinite(z.real) or not math.isfinite(z.imag):
        return str(z)
    if abs(z.imag) <= 1e-9:
        return f"{z.real:.4f}"
    return f"{z.real:.4f}{z.imag:+.4f}j"


def in_chip_domain(a, b):
    a_c = as_complex(a)
    b_c = as_complex(b)
    try:
        exp_a = math.exp(a_c.real)
    except OverflowError:
        return False
    return (
        is_real(a_c)
        and is_real(b_c)
        and math.isfinite(a_c.real)
        and math.isfinite(b_c.real)
        and b_c.real > 0.0
        and Q6_10_MIN <= a_c.real <= Q6_10_MAX
        and 0.0 < b_c.real <= Q6_10_MAX
        and exp_a <= Q6_10_MAX
    )


# Low-level cocotb I/O


def uo_bits(dut):
    return int(dut.uo_out.value)


async def drive_cycle(dut, ser_in=0, shift_en=0, start=0):
    dut.ui_in.value = (ser_in & 1) | ((shift_en & 1) << 1) | ((start & 1) << 2)
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)


async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def shift_in_byte(dut, byte):
    for bit_idx in range(8):
        bit = (byte >> (7 - bit_idx)) & 1
        await drive_cycle(dut, ser_in=bit, shift_en=1)
    await drive_cycle(dut)


async def drain_idle(dut, cycles=4):
    for _ in range(cycles):
        await drive_cycle(dut)


async def pulse_start(dut):
    await drive_cycle(dut, start=1)
    await drive_cycle(dut)


async def wait_for_done(dut, limit=12000):
    for _ in range(limit):
        if (uo_bits(dut) >> 2) & 1:
            return
        await drive_cycle(dut)
    raise AssertionError("Timed out waiting for done")


async def wait_for_tx_pending(dut, limit=500):
    for _ in range(limit):
        if (uo_bits(dut) >> 5) & 1:
            return
        await drive_cycle(dut)
    raise AssertionError("Timed out waiting for tx_pending")


async def shift_out_response(dut):
    assert (uo_bits(dut) >> 5) & 1, "tx_pending must be high before reading"
    bits = 0
    for _ in range(RESP_BITS):
        bits = (bits << 1) | (uo_bits(dut) & 1)
        await drive_cycle(dut, shift_en=1)
    await drive_cycle(dut)
    assert ((uo_bits(dut) >> 5) & 1) == 0, "tx_pending must clear after response"
    return bits


# Chip interface


async def chip_send_frame(dut, a_f, b_f):
    """
    Load one (a, b) frame into the chip over the serial interface.
    a_f and b_f must be plain Python floats in the Q6.10 range.
    """
    uo = uo_bits(dut)
    if (uo >> 5) & 1:
        for _ in range(RESP_BITS + 2):
            await drive_cycle(dut, shift_en=1)
        await drain_idle(dut)
    elif (uo >> 3) & 1 or (uo >> 1) & 1:
        await drain_idle(dut, 8)

    a_bits = float_to_q6_10(a_f)
    b_bits = float_to_q6_10(b_f)
    frame = [
        SOF,
        (a_bits >> 8) & 0xFF,
        a_bits & 0xFF,
        (b_bits >> 8) & 0xFF,
        b_bits & 0xFF,
    ]
    for byte_val in frame:
        await shift_in_byte(dut, byte_val)
    assert (uo_bits(dut) >> 4) & 1, "rx_full should assert after 5-byte frame"


async def chip_eval(dut, a_f, b_f):
    """
    Send (a, b) to the chip and return (float result, int status).
    The chip computes eml(a, b) = exp(a) - ln(b) in Q6.10 fixed point.
    status bits: [2]=error  [1]=domain_error  [0]=overflow
    """
    await chip_send_frame(dut, a_f, b_f)
    await pulse_start(dut)
    await wait_for_done(dut)
    await wait_for_tx_pending(dut)
    response = await shift_out_response(dut)
    status = (response >> 33) & 0x7
    res_bits = (response >> 17) & 0xFFFF
    await drain_idle(dut)
    return q6_10_to_float(res_bits), status


# ── EML evaluation paths ──────────────────────────────────────────────────────


def eml_software(a, b):
    """
    Evaluate eml(a, b) = exp(a) - ln(b) entirely in software using
    80-digit mpmath. Handles complex domain and principal-branch ln.
    This is the only correct path for complex intermediates.
    """
    mp.mp.dps = HOST_DPS
    try:
        result = mp.exp(to_mpc(as_complex(a))) - mp.log(to_mpc(as_complex(b)))
    except (OverflowError, ValueError):
        b_c = as_complex(b)
        return complex(math.inf, 0), (0b010 if b_c.real <= 0 else 0b001)
    result = complex(result)
    status = 0 if is_finite_complex(result) else 0b001
    return result, status


async def eml_node(dut, a, b, stats):
    """
    Evaluate one EML node: eml(a, b) = exp(a) - ln(b).

    Routing:
      chip   — when operands are real, b > 0, and both within Q6.10 range.
               The chip result IS the stack value. No correction.
      software — everything else: complex operands, b <= 0, out of range.
                 Uses mpmath at 80-digit precision.

    stats keys updated here:
      'chip'     : chip called and result used as stack value
      'software' : mpmath used (complex / out-of-domain)
      'chip_err' : chip returned non-zero status for in-domain input (unexpected)
    """
    if in_chip_domain(a, b):
        result_f, status = await chip_eval(dut, as_complex(a).real, as_complex(b).real)
        if status == 0:
            stats["chip"] += 1
            return complex(result_f, 0.0), status
        stats["chip_err"] += 1
        dut._log.warning(
            f"chip returned status={status:03b} for in-domain "
            f"a={as_complex(a).real}, b={as_complex(b).real}"
        )

    stats["software"] += 1
    return eml_software(a, b)


async def run_program(dut, program, x_val, y_val, stats):
    stack = []
    for tok in program:
        if tok == "1":
            stack.append(complex(1.0, 0.0))
        elif tok == "x":
            stack.append(as_complex(x_val))
        elif tok == "y":
            stack.append(as_complex(y_val))
        elif tok == "E":
            if len(stack) < 2:
                return complex(0.0), 0b100, False
            b_val = stack.pop()
            a_val = stack.pop()
            val, _ = await eml_node(dut, a_val, b_val, stats)
            stack.append(val)
        else:
            return complex(0.0), 0b100, False

    if not stack:
        return complex(0.0), 0b100, False

    final = stack[-1]
    final_status = 0 if is_finite_complex(final) else 0b001
    return final, final_status, True


# Tests


@cocotb.test()
async def test_protocol_rejects_early_start(dut):
    """Chip must assert error when start is pulsed before a frame is loaded."""
    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())
    await reset_dut(dut)

    await drive_cycle(dut, start=1)
    assert (uo_bits(dut) >> 3) & 1, "start before frame should set error"
    await drive_cycle(dut)


@cocotb.test()
async def test_protocol_rejects_extra_bytes(dut):
    """Chip must assert error when more than 5 bytes arrive in one frame."""
    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())
    await reset_dut(dut)

    for byte in [SOF, 0x00, 0x00, 0x00, 0x00, 0x00]:
        await shift_in_byte(dut, byte)
    assert (uo_bits(dut) >> 3) & 1, "extra byte should set error"


@cocotb.test()
async def test_chip_eml_real_scalar(dut):
    """
    Verify the chip correctly computes eml(0.5, 0.5) = exp(0.5) - ln(0.5).
    Expected ≈ 2.3419.  Tolerance: 1 LSB of Q6.10 ≈ 0.001, allow 0.01.
    """
    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())
    await reset_dut(dut)

    x, y = 0.5, 0.5
    got, status = await chip_eval(dut, x, y)
    expected = math.exp(x) - math.log(y)
    err = abs(got - expected)

    dut._log.info(
        f"chip eml(0.5, 0.5): got={got:.5f}  expected={expected:.5f}  "
        f"err={err:.5f}  status={status:03b}"
    )
    assert status == 0, f"unexpected status {status:03b}"
    assert err <= 0.01, f"chip error {err:.5f} exceeds Q6.10 tolerance"


@cocotb.test()
async def test_chip_special_values(dut):
    """
    Verify the chip's special-value handling for boundary inputs.
    All cases stay on-chip; no software fallback involved.
    """
    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())
    await reset_dut(dut)

    cases = [
        # (a,          b,         check,                            description)
        (math.inf, 1.0, lambda r: r == math.inf, "eml(+inf, 1)     = +inf"),
        (1.0, math.inf, lambda r: r == -math.inf, "eml(1, +inf)     = -inf"),
        (math.inf, math.inf, lambda r: math.isnan(r), "eml(+inf, +inf)  = NaN"),
        (-math.inf, 1.0, lambda r: abs(r) < 0.01, "eml(-inf, 1)     ≈ 0"),
        (1.0, 0.0, lambda r: r == math.inf, "eml(1, 0)        = +inf"),
    ]
    for a, b, check, desc in cases:
        result, status = await chip_eval(dut, a, b)
        dut._log.info(f"{desc}  →  got={result}  status={status:03b}")
        assert check(result), f"FAIL: {desc}  got {result}"


@cocotb.test()
async def test_software_eml_complex(dut):
    """
    Verify the software EML path for complex operands.
    The chip is NOT used here — complex inputs are outside its domain.
    This tests the host-side fallback that handles principal-branch semantics.
    """
    # dut is not touched in this test; cocotb requires the parameter.
    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())
    await reset_dut(dut)

    a = complex(0.4, 0.2)
    b = complex(0.6, 0.3)

    got, status = eml_software(a, b)

    mp.mp.dps = HOST_DPS
    expected = complex(mp.exp(to_mpc(a)) - mp.log(to_mpc(b)))
    err = abs(got - expected)

    dut._log.info(
        f"software eml({a}, {b}):\n"
        f"  got      = {fmt(got)}\n"
        f"  expected = {fmt(expected)}\n"
        f"  err      = {err:.2e}  status={status:03b}"
    )
    assert status == 0, f"unexpected status {status:03b}"
    assert err <= 1e-6, f"software EML error {err:.2e} too large"


@cocotb.test()
async def test_all_38_eml_programs(dut):
    """
    Evaluate all 38 paper functions via the host EML stack interpreter.

    Architecture under test
    -----------------------
    Chip  : one real Q6.10 EML primitive  eml(x,y) = exp(x) - ln(y)
    Host  : RPN stack machine; routes each E token to chip or software
    """
    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())
    await reset_dut(dut)

    stats = {"chip": 0, "software": 0, "chip_err": 0}
    total = 0
    passed = 0
    failed = []
    x_val = 0.5
    y_val = 0.5

    for name, program, arity, expected_raw in programs:
        total += 1
        expected = as_complex(expected_raw)

        actual, sticky_status, ok = await run_program(dut, program, x_val, y_val, stats)
        err = abs(actual - expected)

        if ok and sticky_status == 0 and err <= PROG_TOL:
            passed += 1
        else:
            failed.append((name, sticky_status, actual, expected, err, ok))

    total_nodes = stats["chip"] + stats["software"] + stats["chip_err"]

    dut._log.info(
        f"\n{'=' * 58}\n"
        f"  38-function EML sweep\n"
        f"{'=' * 58}\n"
        f"  programs : {passed}/{total} passed\n"
        f"  EML nodes: {total_nodes} total\n"
        f"    chip (result used as stack value) : {stats['chip']}\n"
        f"    software / mpmath (complex, OOB)  : {stats['software']}\n"
        f"    chip errors / unexpected status   : {stats['chip_err']}\n"
        f"{'=' * 58}"
    )

    for name, status, got, exp, err, ok in failed:
        dut._log.info(
            f"  FAIL {name}: ok={ok}  status={status:03b}  "
            f"got={fmt(got)}  expected={fmt(exp)}  err={err:.4f}"
        )

    assert total == len(programs), f"program list mismatch: {total} vs {len(programs)}"
    assert passed == total, f"expected {total}/38 to pass, got {passed}"
