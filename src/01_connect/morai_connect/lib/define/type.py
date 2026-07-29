import ctypes

_bool = ctypes.c_bool #1
_byte = ctypes.c_byte #1
_char = ctypes.c_char #1
_short = ctypes.c_short #2

_float = ctypes.c_float #4
_double = ctypes.c_double #8

_int = ctypes.c_int #4
_int8 = ctypes.c_int8 #1
_int16 = ctypes.c_int16 #2 
_int32 = ctypes.c_int32 #4
_int64 = ctypes.c_int64 #8

_uint = ctypes.c_uint #4
_uint8 = ctypes.c_uint8 #1
_uint16 = ctypes.c_uint16 #2
_uint32 = ctypes.c_uint32 #4
_uint64 = ctypes.c_uint64 #8


__all__ = [
    "_bool", "_byte", "_char", "_short",
    "_float", "_double",
    "_int", "_int8", "_int16", "_int32", "_int64",
    "_uint", "_uint8", "_uint16", "_uint32", "_uint64"
]