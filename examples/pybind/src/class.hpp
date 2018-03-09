
struct Pet {
    Pet(const std::string &name, int age) : name(name), age(age) {}
    
    void set(int age_) { age = age_; }
    void set(const std::string &name_) { name = name_; }

    void setName(const std::string &name_) { name = name_;}
    const std::string &getName() const { return name; }

    std::string name;
    int age;
};

struct Dog : Pet {
    Dog(const std::string &name) : Pet(name, 0) {}
    std::string bark() const { return "woof!"; }
};
