# Fix-point Decimal Type

A simple fix point decimal type implementation where the precision is part of the type. 
Define your number type by importing the whole module (`fp_decimal_type::*`) and invoking 

```define_fpd_type(name: <type_name>, type: <underlying_signed_integer_type>, precision: <precision_digits>)```

for signed types, and 

```define_unsigned_fpd_type(name: <type_name>, type: <underlying_unsigned_integer_type>, precision: <precision_digits>)` for unsigned types.```



### Limitations

- Conversions via the From trait and basic aritmetic operations are only defined for primitive types, not between different fpd types -- you need to define them yourself for now
- type declaration happens in your project via a macro -- required imports are reexported through the package and may pollute your namespace
- precision is capped at 10 precision digits (can be increased by expanding the POW_10 const array)