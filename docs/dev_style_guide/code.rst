Code Style Guide
================

Variable names are written in snake case, with all lowercase letters and underscores between words

e.g. ```uint16_t this_is_snake_case = 0```

Function names are written in camel case, in accordance with the Arduino style guide.

e.g. ```void thisIsCamelCase();```

Class names are written similar to camel case, but with the additional restrictions:

* All EVN-specific classes start with ``EVN``
* Similar abbreviations with all uppercase letters (e.g. ``LED``) remain uppercase
* Class names longer than 18 characters should be shortened.

All hardware classes should have a ``begin()`` function.

Pointers are to be avoided unless **absolutely necessary**.

Apart from this, there are no further guidelines for standardising code (yet).