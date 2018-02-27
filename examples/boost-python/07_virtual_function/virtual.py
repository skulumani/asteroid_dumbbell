#!/usr/bin/env python3

from virtual import Base, identify

class PythonDerived(Base):
    def name(self):
        return "PythonDerived"

b = Base()
identify(b)

p = PythonDerived()
identify(p)
