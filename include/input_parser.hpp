#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

#include <string>
#include <vector>
// InputParser definition

class InputParser {

    public:
        InputParser (int &argc, char **argv);
        const std::string& get_command_option(const std::string &option) const;
        bool option_exists(const std::string &option) const;
    private:
        std::vector <std::string> tokens;
};

#endif
