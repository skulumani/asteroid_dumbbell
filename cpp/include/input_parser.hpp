#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

// InputParser definition
#include <string>
#include <algorithm>
#include <vector>

class InputParser {

    public:
        InputParser (int &argc, char **argv);
        const std::string& get_command_option(const std::string &option) const;
        bool option_exists(const std::string &option) const;
    private:
        std::vector <std::string> tokens;
};

#endif
