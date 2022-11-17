#include <dirent.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdio.h>

#include <list>
#include <string>

#include "common/application/HelloWorldInterface.h"
#include "gtest/gtest.h"

// Grab the file list
std::list<std::string> get_files(const std::string& path) {
  std::list<std::string> files;

  DIR* d;
  struct dirent* dir;
  d = opendir(path.c_str());
  if(d) {
    while((dir = readdir(d)) != NULL) {
      if(dir->d_type == DT_REG) {
        files.push_back(dir->d_name);
      }
    }
    closedir(d);
  }

  return files;
}

TEST(DynamicLibraryTesting, SanityCheckAllApplicationsInLib) {
  const std::string path = "../lib";

  // Iterate over all the files in the directory
  for(auto file : get_files(path)) {
    auto fullpath = path + "/" + file;
    printf("\nTesting shared object: %s\n", fullpath.c_str());

    // Load the object
    auto dso = dlopen(fullpath.c_str(), RTLD_NOW);
    ASSERT_NE(dso, nullptr) << dlerror();

    // Validate known invalid methods
    for(auto& method : {"ctest1", "makeUpName"}) {
      EXPECT_EQ(dlsym(dso, method), nullptr) << dlerror();
    }

    // Validate known valid method
    for(auto& method : {"hello_world"}) {
      auto fn = dlsym(dso, method);
      ASSERT_NE(fn, nullptr) << dlerror();

      auto hello = reinterpret_cast<int (*)(void)>(fn);
      EXPECT_GT(hello(), 1000);
      printf("My hello world number is %d\n", hello());
    }

    // Validate known valid method the produces an concrete instance of a class
    for(auto& method : {"newInstance"}) {
      auto fn = dlsym(dso, method);
      ASSERT_NE(fn, nullptr) << dlerror();

      char* error;
      auto instance = reinterpret_cast<std::unique_ptr<HelloWorldInterface> (*)(void)>(dlsym(dso, "newInstance"));
      ASSERT_EQ((error = dlerror()), nullptr) << error;
      EXPECT_GT(instance()->get_list().size(), 0u);

      // Print out some of the methods information
      for(auto& l : instance()->get_list()) {
        printf("%s ", l.c_str());
      }
      printf("    ---    ");
      instance()->print();
      printf("\n");
    }

    // Release the object
    EXPECT_EQ(dlclose(dso), 0u) << dlerror();
  }
}
