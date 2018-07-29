#include <stdio.h>
#include <fstream>
#include <iostream>

//#include <ros/ros.h>
#include <smpl/csv_parser.h>

int main(int argc, char* argv[])
{
//    ros::init(argc, argv, "csv_parser_test");
//    ros::NodeHandle nh;

    if (argc < 2) {
        printf("Usage: csv_parser_test <path>\n");
        return 0;
    }
    std::ifstream ifs(argv[1]);

    smpl::CSVParser parser;
    if (!parser.parseStream(ifs)) {
        fprintf(stderr, "failed to parse %s\n", argv[0]);
        return 1;
    }

    for (size_t i = 0; i < parser.recordCount(); ++i) {
        for (size_t j = 0; j < parser.fieldCount(); ++j) {
            std::cout << parser.fieldAt(i, j) << ", ";
        }
        std::cout << '\n';
    }

    return 0;
}
