#include <string>
#include <iostream>
#include "H5Cpp.h"

const int MAX_NAME_LENGTH = 32;
const std::string Filename("SimpleCompound.hdf5");
const std::string DatasetName("PersonalInformation");
const std::string member_age("Age");
const std::string member_sex("Sex");
const std::string member_name("Name");
const std::string member_height("Height");

struct PersonalInformation {
    int age;
    char sex;
    char name[MAX_NAME_LENGTH];
    float height;
};

int main( void) {
    // Data to write to file
    PersonalInformation person_list[] = {
        {21, 'M', "Paul", 152.0},
        {23, 'M', "Ringo", 155.0},
        {23, 'M', "John", 155.0},
        {20, 'M', "George", 156.0}
    };

    // length of data
    int length = sizeof(person_list) / sizeof(PersonalInformation);
    // array of each length of data
    hsize_t dim[1];
    dim[0] = sizeof(person_list) / sizeof(PersonalInformation);

    // length of dim
    int rank = sizeof(dim) / sizeof(hsize_t);

    // define the datatype
    H5::CompType mtype(sizeof(PersonalInformation));
    mtype.insertMember(member_age, HOFFSET(PersonalInformation, age), H5::PredType::NATIVE_INT);
    mtype.insertMember(member_sex,  HOFFSET(PersonalInformation, sex), H5::PredType::C_S1);
    mtype.insertMember(member_name, HOFFSET(PersonalInformation, name), H5::StrType(H5::PredType::C_S1, MAX_NAME_LENGTH));
    mtype.insertMember(member_height, HOFFSET(PersonalInformation, height), H5::PredType::NATIVE_FLOAT);

    // dataset and file
    H5::DataSpace space(rank, dim);
    H5::H5File *file = new H5::H5File(Filename, H5F_ACC_TRUNC);
    H5::DataSet *dataset = new H5::DataSet(file->createDataSet(DatasetName, mtype, space));
    // write the data
    dataset->write(person_list, mtype);
    
    delete dataset;
    delete file;

    return 0;
}
