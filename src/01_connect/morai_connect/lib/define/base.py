import ctypes


class Base(ctypes.Structure):
    _pack_ = 1 

    def __repr__(self):
        field_strings = []
        append_field = field_strings.append 

        for field_name, field_type in self._fields_:
            value = getattr(self, field_name) if isinstance(field_type, ctypes.Array) else getattr(self, field_name)            
            if isinstance(value, bytes):
                value = value.decode('utf-8')
            elif isinstance(value, ctypes.Array):  # If it's an array, convert to list
                value = list(value)
            append_field(f"{field_name}: {value}")
        
        return "\n".join(field_strings)