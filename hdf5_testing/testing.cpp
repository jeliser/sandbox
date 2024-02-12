#if 0
#include <iostream>
#include <vector>
#include <string>
#include <H5Cpp.h>

int main() {
    // Define the number of rows and columns
    const int num_rows = 5;
    const int num_cols = 4;

    // Define the size of each datatype
    const int sizes[num_cols] = {sizeof(uint8_t), sizeof(float), sizeof(int32_t), sizeof(double)};
    uint32_t row_size = sizes[0] + sizes[1] + sizes[2] + sizes[3];

    // Create a raw buffer and fill it with data
    char* buffer = new char[num_rows * row_size];

    // Fill the buffer with data here
    for(uint32_t i = 0; i < num_rows; ++i) {
        *reinterpret_cast<uint8_t*>(&buffer[(i*row_size) + 0]) = i;
        *reinterpret_cast<float*>(&buffer[(i*row_size) + 1]) = static_cast<float>(i);
        *reinterpret_cast<uint32_t*>(&buffer[(i*row_size) + 5]) = i;
        *reinterpret_cast<double*>(&buffer[(i*row_size) + 9]) = i;
        //*reinterpret_cast<double*>(&buffer[(i*row_size) + 9]) = 1353989571.120000;
    }

    // Open HDF5 file
    hid_t hdf5_file = H5Fcreate("mytestfile.hdf5", H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    // Create a dataspace
    hsize_t dims[1] = {num_rows};  // Number of records
    hid_t dataspace = H5Screate_simple(1, dims, NULL);

    // Create the compound datatype
    hid_t table_type = H5Tcreate(H5T_COMPOUND, sizes[0] + sizes[1] + sizes[2] + sizes[3]);

    // Insert the fields into the compound datatype
    size_t offset = 0;
    H5Tinsert(table_type, "My_column.1", offset, H5T_NATIVE_UINT8);
    offset += sizes[0];
    H5Tinsert(table_type, "My_column.2", offset, H5T_NATIVE_FLOAT);
    offset += sizes[1];
    H5Tinsert(table_type, "My_column.3", offset, H5T_NATIVE_INT32);
    offset += sizes[2];
    H5Tinsert(table_type, "My_column.4", offset, H5T_NATIVE_DOUBLE);

    // Create a dataset
    hid_t dataset = H5Dcreate2(hdf5_file, "dataset", table_type, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    // Write the buffer to the dataset
    H5Dwrite(dataset, table_type, H5S_ALL, H5S_ALL, H5P_DEFAULT, buffer);

    // Close the dataset, dataspace, and hdf5_file
    H5Dclose(dataset);
    H5Sclose(dataspace);
    H5Fclose(hdf5_file);

    // Delete the buffer
    delete[] buffer;

    return 0;
}


#else

#include <iostream>
#include <vector>
#include <string>
#include <H5Cpp.h>

int main() {
    // Define the number of rows and columns
    const int num_rows = 5;
    const int num_cols = 4;

    // Create a 2D vector of doubles and fill it with data
    std::vector<std::vector<double>> data(num_rows, std::vector<double>(num_cols));
    for(int i = 0; i < num_rows; ++i) {
        for(int j = 0; j < num_cols; ++j) {
            data[i][j] = static_cast<double>(i * num_cols + j);
        }
    }

    // Flatten the 2D vector into a 1D vector
    std::vector<double> flat_data;
    for(const auto& row : data) {
        flat_data.insert(flat_data.end(), row.begin(), row.end());
    }

    // Open HDF5 file
    hid_t hdf5_file = H5Fcreate("mytestfile.hdf5", H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    // Create a dataspace
    hsize_t dims[2] = {num_rows, num_cols};
    hid_t dataspace = H5Screate_simple(2, dims, NULL);

    // Create a dataset
    hid_t dataset = H5Dcreate2(hdf5_file, "dataset", H5T_NATIVE_DOUBLE, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    // Write the data to the dataset
    H5Dwrite(dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, flat_data.data());

    // Close the dataset, dataspace, and hdf5_file
    H5Dclose(dataset);
    H5Sclose(dataspace);
    H5Fclose(hdf5_file);

    return 0;
}

#endif

