"""Scheduling hint only; it cannot change any numerical result.

On hybrid Intel laptops Windows 11 may confine background worker processes to
the efficiency cores. A worker can ask not to be power-throttled, which lets
the scheduler use the idle performance cores. No-op on other platforms.
"""
import ctypes
import sys


def full_speed():
    if sys.platform != 'win32':
        return False
    try:
        class PowerThrottling(ctypes.Structure):
            _fields_ = [('Version', ctypes.c_ulong), ('ControlMask', ctypes.c_ulong), ('StateMask', ctypes.c_ulong)]
        kernel = ctypes.windll.kernel32
        kernel.GetCurrentProcess.restype = ctypes.c_void_p
        kernel.SetProcessInformation.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_ulong]
        state = PowerThrottling(1, 1, 0)  # control execution speed, throttling off
        return bool(kernel.SetProcessInformation(kernel.GetCurrentProcess(), 4, ctypes.byref(state), ctypes.sizeof(state)))
    except Exception:
        return False
