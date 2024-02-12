#include "H5Cpp.h"
#include <vector>
#include <fstream>

// Assuming that Data is a struct that represents a row of data
struct Data {
    int id;
    float value;
};

// Function to generate a row of data
Data generate_data(int id, float value) {
    Data data;
    data.id = id;
    data.value = value;
    return data;
}

int main() {

    const std::string message_name = "test";
    size_t num_rows = 9;
    size_t rows_per_chunk = 3;

    // Generate data for each row
    std::vector<Data> data;
    for (int i = 0; i < num_rows; i += rows_per_chunk) {
        for (int j = 0; j < rows_per_chunk; ++j) {
            data.push_back(generate_data(((i/rows_per_chunk)*rows_per_chunk) + j, (((i/rows_per_chunk)*rows_per_chunk) + j) * 1.5f));
        }
    }

    // chuckerize the data saving
    for (int i = 0; i < num_rows; i += rows_per_chunk) {
        std::string filename = message_name + ".hdf5";
        std::ifstream file(filename.c_str());

        // Check if the file already exists
        hid_t hdf5_file;
        if (file) {
            // File exists, open it
            hdf5_file = H5Fopen(filename.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
        } else {
            // File doesn't exist, create it
            hdf5_file = H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
        }

        // Check if the group exists
        hid_t group;
        if (H5Lexists(hdf5_file, (std::string("/") + message_name).c_str(), H5P_DEFAULT) > 0) {
            // Group exists, open it
            group = H5Gopen2(hdf5_file, (std::string("/") + message_name).c_str(), H5P_DEFAULT);
        } else {
            // Group doesn't exist, create it
            group = H5Gcreate2(hdf5_file, (std::string("/") + message_name).c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        }

        // Create a type for the struct
        hid_t datatype = H5Tcreate(H5T_COMPOUND, sizeof(Data));
        H5Tinsert(datatype, "id", HOFFSET(Data, id), H5T_NATIVE_INT);
        H5Tinsert(datatype, "value", HOFFSET(Data, value), H5T_NATIVE_FLOAT);

        // Define the size of the dataspace
        hsize_t dims[1] = {rows_per_chunk};
        hsize_t max_dims[1] = {H5S_UNLIMITED}; // Set the maximum size to unlimited

        // Create the dataspace for the dataset
        hid_t dataspace = H5Screate_simple(1, dims, max_dims);

        // Try to open the dataset
        hid_t dataset;
        if (H5Lexists(group, "dataset", H5P_DEFAULT) > 0) {
            // Dataset exists, open it
            dataset = H5Dopen2(group, "dataset", H5P_DEFAULT);

            // Get the current size of the dataset
            hid_t old_dataspace = H5Dget_space(dataset);
            H5Sget_simple_extent_dims(old_dataspace, dims, NULL);

            // Calculate the new size of the dataset
            dims[0] += rows_per_chunk;

            // Extend the dataset
            H5Dset_extent(dataset, dims);

            // Close the old dataspace
            H5Sclose(old_dataspace);
        } else {
            // Create a property list for chunking
            hid_t plist = H5Pcreate(H5P_DATASET_CREATE);
            hsize_t chunk_dims[1] = {std::min(rows_per_chunk, static_cast<size_t>(2))};
            H5Pset_chunk(plist, 1, chunk_dims);

            // Create the dataset in the group with chunking
            dataset = H5Dcreate2(group, "dataset", datatype, dataspace, H5P_DEFAULT, plist, H5P_DEFAULT);

            // Close the property list
            H5Pclose(plist);
        }

        // Get the dataspace of the extended dataset
        hid_t extended_dataspace = H5Dget_space(dataset);

        // Select the extended part of the dataset
        hsize_t start[1] = {dims[0] - rows_per_chunk};
        hsize_t count[1] = {rows_per_chunk};
        H5Sselect_hyperslab(extended_dataspace, H5S_SELECT_SET, start, NULL, count, NULL);

        // Write the data to the dataset
        H5Dwrite(dataset, datatype, dataspace, extended_dataspace, H5P_DEFAULT, &data[i]);


        // Close the dataspace
        H5Sclose(dataspace);
        H5Dclose(dataset);
        H5Tclose(datatype);
        H5Gclose(group);
        H5Fclose(hdf5_file);
    }

    return 0;
}