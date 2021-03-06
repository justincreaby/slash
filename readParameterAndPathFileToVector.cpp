#include "readParameterAndPathFileToVector.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

void readParameterFile(std::string parameterFileName, std::vector<double> &parameterValues)
{
    std::ifstream parameterFile;

    parameterFile.open (parameterFileName);   /* open file with filename as argument */
    if (! parameterFile.is_open()) /* validate file open for reading */
    {    
        std::cerr << "error: file open failed '" << parameterFileName << "'.\n";
    }
    
    std::string line, val;                  /* string for line & value */
    int fileValueIndex = 1;
    
    while (std::getline (parameterFile, line))   /* read each line */
    {
        std::stringstream s (line);         /* stringstream line */
        while (std::getline (s, val, ':'))  /* get each value (': ' delimited) */
        {
            if (fileValueIndex % 2 == 0)
            {
                parameterValues.push_back (std::stod (val));
            }
            fileValueIndex++;
        }
    }
}

void readPathFile(std::string pathFileName, std::vector<double> &pathX, std::vector<double> &pathY, std::vector<double> &pathVelocity) 
{

    std::ifstream pathFile;

    pathFile.open (pathFileName);   /* open file with filename as argument */
    if (! pathFile.is_open()) /* validate file open for reading */
    {    
        std::cerr << "error: file open failed '" << pathFileName << "'.\n";
    }
    
    std::string line, val;                  /* string for line & value */
    //std::vector<double> pathX;
    //std::vector<double> pathY;
    int fileValueIndex = 1;
    
    while (std::getline (pathFile, line))   /* read each line */
    {
        std::stringstream s (line);         /* stringstream line */
        while (std::getline (s, val, ','))  /* get each value (',' delimited) */
        {
            //std::cout << "Value(" << val << "), fileValueIndex(" << fileValueIndex << ")\n";
            if (1 == fileValueIndex)
            {
                pathX.push_back (std::stod (val));
            }
            if (2 == fileValueIndex)
            {
                pathY.push_back (std::stod (val));
                //std::cout << std::stod (val) << "\n";
            }
            if (3 == fileValueIndex)
            {
                pathVelocity.push_back (std::stod (val));
                //std::cout << std::stod (val) << "\n";
            }

            fileValueIndex++;
            if (fileValueIndex > 3)
            {
                fileValueIndex = 1;
            }
        }
    }
        
    // for (int i=0; i<pathX.size(); i++)
    // {
    //     std::cout << pathX[i] << " " << pathY[i] << "\n";
    // }
}