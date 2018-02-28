/**
    Read OBJ file to CGAL Mesh/Eigen arrays

    @author Shankar Kulumani
    @version 0.1 2/28/2018
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

// definition for the function
bool loadOBJ (const std::string path, std::vector<double> &vertices, std::vector<double> &faces);


class InputParser {

    public:
        InputParser (int &argc, char **argv) {
            for (int ii = 1; ii < argc; ++ii) {
                this->tokens.push_back(std::string(argv[ii]));
            }
        }
        // only allows reading from object not WRITING
        const std::string& get_command_option(const std::string &option) const {
            std::vector<std::string>::const_iterator itr;
            itr = std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }

        bool option_exists(const std::string &option) const {
            return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
        }

    private:
        std::vector <std::string> tokens;

};

int main(int argc, char* argv[]) {
    InputParser input(argc, argv);
    if (input.option_exists("-h")) {
        std::cout << "Usage read_obj -i input_file.obj" << std::endl;
    }

    const std::string &input_file = input.get_command_option("-i");
    if (!input_file.empty()) {
        std::cout << "Filename = " << input_file << std::endl;
    }
    /* std::cout << "path is " << ipath << std::endl; */
    /* std::cout << argv[0] << argv[1] << std::endl; */
    /* std::cout << argc << std::endl; */
    /* std::string pathname = "cube.obj"; */

    /* std::ifstream ifile(pathname); */

    /* if (!ifile.is_open()) { */
    /*     std::cout << "Could not open " << pathname << std::endl; */
    /*     return 1; */
    /* } */

    /* std::string line; */    
    /* while (std::getline(ifile, line)) { */
    /*     std::cout << line << std::endl; */
    /* } */
    
    return 0;
}
