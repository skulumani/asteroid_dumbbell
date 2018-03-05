/**
    Parse command line inputs 
    
    This was found on StackOverflow - I'll find the link eventually

    @author Shankar Kulumani
    @version 5 March 2018
*/
#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

#include <string>
#include <vector>

/**
    This class sets up the input parser. Can test for arguments and find
    options associated with an argument

    @param argc Standard count of arguments
    @param argv Array of argument variables
*/
class InputParser {

    public:
        InputParser (int &argc, char **argv);

        /**
            Get option associated with an argument

            main -i input_file.txt

            Will return input_file.txt

            @param option Name of argument to find
            @returns string The option associated with the argument
        */
        const std::string& get_command_option(const std::string &option) const;

        /**
            Test if an argument exists

            @param option Argument to test if true
            @returns boolean True or False if the argument was passed
        */
        bool option_exists(const std::string &option) const;
    private:
        std::vector <std::string> tokens;
};

#endif
