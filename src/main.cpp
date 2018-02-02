

#include <exception>
#include <iostream>
#include <CSV.hpp>
#include <Eigen>

int main()
{
    csv::CsvFile::DefaultSep = ' ';

    try
    {
        csv::CsvFile csvf;
        csvf.load("../data/world.dat");
        std::cout << csvf.size() << ',' << csvf[0].size() << ',' << csvf[0][0].asString() << std::endl;

        csvf.load("../data/sensor_data.dat");
        std::cout << csvf.size() << ',' << csvf[0].size() << ',' << csvf[0][0].asString() << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }


    return 0;
}
