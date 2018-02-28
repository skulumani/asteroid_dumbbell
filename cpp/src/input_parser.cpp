#include "input_parser.hpp"

// definitions for input parser
// constructor definition
InputParser::InputParser (int &argc, char **argv) {
    for (int ii = 1; ii < argc; ++ii) {
        this->tokens.push_back(std::string(argv[ii]));
    }
}
// only allows reading from object not WRITING
const std::string& InputParser::get_command_option(const std::string &option) const {
    std::vector<std::string>::const_iterator itr;
    itr = std::find(this->tokens.begin(), this->tokens.end(), option);
    if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
        return *itr;
    }
    static const std::string empty_string("");
    return empty_string;
}

bool InputParser::option_exists(const std::string &option) const {
    return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
}


