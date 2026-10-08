"""LOCAL_READ02 §2: a past run of equal command / measured values must not block a later, new independent residual."""
from sweepick.control import sweepick_gripper_contact_hold as tg

DT = 0.02


def moving_equal(j, n=22, start=1.0):
    t, c = 0.0, start
    for i in range(n):
        t, c = t + DT, c - 0.01
        j.update(t, c, c, seq=i)
    return t, c


def test_new_independent_residual_after_no_discrimination_is_evaluated_again():
    j = tg.Jaw(tg.SIM_PGRIPPER, DT, provenance=tg.INDEPENDENT)
    t, c = moving_equal(j)
    assert j.state == tg.NO_DISCRIMINATION
    for i in range(30):                                                       # new stamps / seq, command at rest, residual 3e-5
        t += DT
        s = j.update(t, c, c + 3e-5, seq=100 + i)
    assert s == tg.CONTACT_CANDIDATE and j.loaded_s >= tg.SIM_PGRIPPER.confirm_s
    assert j.recoveries == 1 and not j.closed_and_resting()                   # a contact candidate is not a confirmed hold


def test_re_reading_the_same_sequence_does_not_recover():
    j = tg.Jaw(tg.SIM_PGRIPPER, DT, provenance=tg.INDEPENDENT)
    t, c = moving_equal(j)
    j.update(t + DT, c, c + 3e-5, seq=100)
    for i in range(30):
        s = j.update(t + DT, c, c + 3e-5, seq=100)                            # the same measurement consumed again
    assert s != tg.CONTACT_CANDIDATE and j.loaded_s <= DT + 1e-9 and j.duplicates == 30   # one new sample, then nothing accumulates


def test_equal_values_from_new_bus_reads_are_new_samples_but_no_evidence():
    j = tg.Jaw(tg.SIM_PGRIPPER, DT, provenance=tg.INDEPENDENT)
    t, c = moving_equal(j)
    n = j.samples
    for i in range(30):
        t += DT
        s = j.update(t, c, c, seq=100 + i)
    assert j.samples == n + 30 and j.duplicates == 0 and s == tg.NO_DISCRIMINATION


def test_a_declared_echo_never_recovers():
    j = tg.Jaw(tg.SIM_PGRIPPER, DT, provenance=tg.ECHO)
    t, c = moving_equal(j)
    for i in range(30):
        t += DT
        s = j.update(t, c, c + 3e-5, seq=100 + i)
    assert s == tg.UNAVAILABLE and j.loaded_s == 0.0
