#ifndef MEMBER_H
#define MEMBER_H 

#include <string>

struct SomeClass
{
    SomeClass(std::string name) : name(name), value() {}
    std::string const name; // this is readonly and doesn't chagne
    float value; // this can be modified
};

#endif
