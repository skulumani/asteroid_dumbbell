/**
    Simple wrapper for Eigen to/from HDF5 datasets
    
    Updated/correctd from the work of Jim Garrison
    https://github.com/garrison/eigen3-hdf5

    @author Shankar Kulumani
    @version 23 April 2018
*/

#ifndef EIGEN_HDF5_H
#define EIGEN_HDF5_H
#include <Eigen/Dense>
#include "H5Cpp.h"

#include <stdexcept>
#include <memory>

// TODO Turn on compression by default or maybe with a flag as well
// TODO Add documentation
// TODO Move Each class to it's own file
namespace HDF5 {
    // Ahead of time declaration
    class File;
    class Group;
    class DataSet;
    
    class DataSet {
        public:
            DataSet( void );
            virtual ~DataSet( void );

            // Simply open a dataset with another function to read
            DataSet(const File* file, const std::string& dataset_name);
            DataSet(const Group* group, const std::string& dataset_name);

            // read an already opened dataset to a new variable
            template<typename Derived>
                int read(const Eigen::DenseBase<Derived>& mat);

            // open and read to variable
            template<typename Derived>
                DataSet(const File* file, const std::string& dataset_name, const Eigen::DenseBase<Derived>& mat);
            template<typename Derived>
                DataSet(const Group* group, const std::string& dataset_name, const Eigen::DenseBase<Derived>& mat);


            // create and write to new dataset
            template<typename Derived>
                DataSet(const File* file, const std::string& dataset_name, const Eigen::EigenBase<Derived> &mat,
                        const H5::DSetCreatPropList &plist=H5::DSetCreatPropList::DEFAULT);

            template<typename Derived>
                DataSet(const Group* group, const std::string& dataset_name, const Eigen::EigenBase<Derived> &mat,
                        const H5::DSetCreatPropList &plist=H5::DSetCreatPropList::DEFAULT);


            std::shared_ptr<H5::DataSet> dataset_ptr;
    };
/** @class HDF5Object

    @brief File/Group of HDF5 file
    
    This class is a wrapper for HDF5 groups/files. It has member functions to
    ease the creation of datasets within the object. Already assumes an open 
    file but will create new datasets, groups, or attributes as desired.

    @author Shankar Kulumani
    @version 24 April 2018
*/
class Group {
    // create a new dataset inside the group
    // create attribute in the group
    public:
        Group( void );
        virtual ~Group(void);

        Group(const File* file, const std::string& group_name);
        
        template<typename Derived>
        int write(const std::string& dataset_name, const Eigen::EigenBase<Derived>& mat);

        template<typename Derived>
        int read(const std::string& dataset_name, const Eigen::DenseBase<Derived>& mat);

        // TODO Create dataset method
        std::shared_ptr<H5::Group> group_ptr;
};

class File {
    public: 
        static const int ReadOnly = 0; /**< Read only access */
        static const int ReadWrite = 1; /**< ReadWrite access */
        static const int Truncate = 2; /**< Overwrite a file if it exists or create a new one */
        static const int Excl = 3; /**< Only open if the file doesn't exist */
        static const int Create = 4; /**< Create a new file */


        File( void );

        // close the file
        virtual ~File( void );
        
        /** @fn Open HDF5 file for reading/writing
                
            Can specify some options for the file

            @param file_name File name to open/create
            @param open_flag Choice of mode to open the file in

            @author Shankar Kulumani
            @version 23 April 2018
        */
        File(const std::string& file_name, const int open_flag = File::ReadOnly);
        
        const std::string getName( void ) const;
        
        Group group(const std::string& group_name) const;

        // Open exisiting dataset
        DataSet open_dataset(const std::string& dataset_name) const;

        // Open and read to variable an exisiting dataset
        template<typename Derived>
        DataSet read_dataset(const std::string& dataset_name, const Eigen::DenseBase<Derived> &mat) const;
        
        // create and write to dataset
        template<typename Derived>
        DataSet write_dataset(const std::string& dataset_name, const Eigen::EigenBase<Derived> &mat, const H5::DSetCreatPropList &plist=H5::DSetCreatPropList::DEFAULT) const;

        // TODO Add attribute saving
        // TODO Add saving scalar double, int, strings
        // create a dataset and return HDF5DataSet
        template<typename Derived>
        int write(const std::string& dataset_name, const Eigen::EigenBase<Derived>& mat);
        
        template<typename Derived>
        int read(const std::string& dataset_name, const Eigen::DenseBase<Derived> &mat);

        // create attribute
        
        std::shared_ptr<H5::H5File> file_ptr; /**< HDF5 file to save data */
};

}
template <typename T>
struct DatatypeSpecialization;

// floating-point types

template <>
struct DatatypeSpecialization<float> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_FLOAT;
    }
};

template <>
struct DatatypeSpecialization<double> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_DOUBLE;
    }
};

template <>
struct DatatypeSpecialization<long double> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_LDOUBLE;
    }
};

// integer types

template <>
struct DatatypeSpecialization<short> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_SHORT;
    }
};

template <>
struct DatatypeSpecialization<unsigned short> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_USHORT;
    }
};

template <>
struct DatatypeSpecialization<int> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_INT;
    }
};

template <>
struct DatatypeSpecialization<unsigned int> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_UINT;
    }
};

template <>
struct DatatypeSpecialization<long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_LONG;
    }
};

template <>
struct DatatypeSpecialization<unsigned long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_ULONG;
    }
};

template <>
struct DatatypeSpecialization<long long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_LLONG;
    }
};

template <>
struct DatatypeSpecialization<unsigned long long> {
    static inline const H5::DataType * get (void) {
        return &H5::PredType::NATIVE_ULLONG;
    }
};
// string types, to be used mainly for attributes

template <>
struct DatatypeSpecialization<const char *> {
    static inline const H5::DataType * get (void) {
        static const H5::StrType strtype(0, H5T_VARIABLE);
        return &strtype;
    }
};

template <>
struct DatatypeSpecialization<char *> {
    static inline const H5::DataType * get (void){
        static const H5::StrType strtype(0, H5T_VARIABLE);
        return &strtype;
    }
};

// XXX: for some unknown reason the following two functions segfault if
// H5T_VARIABLE is used.  The passed strings should still be null-terminated,
// so this is a bit worrisome.

template <std::size_t N>
struct DatatypeSpecialization<const char [N]> {
    static inline const H5::DataType * get (void) {
        static const H5::StrType strtype(0, N);
        return &strtype;
    }
};

template <std::size_t N>
struct DatatypeSpecialization<char [N]> {
    static inline const H5::DataType * get (void) {
        static const H5::StrType strtype(0, N);
        return &strtype;
    }
};

template <typename Derived>
void load (const H5::H5Location &h5group, const std::string &name, const Eigen::DenseBase<Derived> &mat);

template <typename Derived>
void save (H5::H5Location &h5group, const std::string &name, 
        const Eigen::EigenBase<Derived> &mat,
        const H5::DSetCreatPropList &plist=H5::DSetCreatPropList::DEFAULT);


namespace internal
{
    template <typename Derived>
    H5::DataSpace create_dataspace (const Eigen::EigenBase<Derived> &mat);

    template <typename Derived>
    bool write_rowmat(const Eigen::EigenBase<Derived> &mat,
        const H5::DataType * const datatype,
        H5::DataSet *dataset,
        const H5::DataSpace* dspace);

    template <typename Derived>
    bool write_colmat(const Eigen::EigenBase<Derived> &mat,
        const H5::DataType * const datatype,
        H5::DataSet *dataset,
        const H5::DataSpace* dspace);

    // H5::Attribute and H5::DataSet both have similar API's, and although they
    // share a common base class, the relevant methods are not virtual.  Worst
    // of all, they take their arguments in different orders!

    template <typename Scalar>
    inline void read_data (const H5::DataSet &dataset, Scalar *data, const H5::DataType &datatype) {
        dataset.read(data, datatype);
    }

    template <typename Scalar>
    inline void read_data (const H5::Attribute &dataset, Scalar *data, const H5::DataType &datatype) {
        dataset.read(datatype, data);
    }

    // read a column major attribute; I do not know if there is an hdf routine to read an
    // attribute hyperslab, so I take the lazy way out: just read the conventional hdf
    // row major data and let eigen copy it into mat. 
    template <typename Derived>
    bool read_colmat(const Eigen::DenseBase<Derived> &mat,
        const H5::DataType * const datatype,
        const H5::Attribute &dataset);

    template <typename Derived>
    bool read_colmat(const Eigen::DenseBase<Derived> &mat,
        const H5::DataType * const datatype,
        const H5::DataSet &dataset);

    template <typename Derived, typename DataSet>
        void _load (const DataSet &dataset, const Eigen::DenseBase<Derived> &mat);
}
#endif
