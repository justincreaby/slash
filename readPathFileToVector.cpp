#include "readPathFileToVector.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

void readPathFile (std::string fileName, std::vector<double> &refPathX, std::vector<double> &refPathY) 
{

    std::ifstream pathFile;

    pathFile.open (fileName);   /* open file with filename as argument */
    if (! pathFile.is_open()) /* validate file open for reading */
    {    
        std::cerr << "error: file open failed '" << fileName << "'.\n";
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
            if (fileValueIndex % 2 == 0)
            {
                refPathY.push_back (std::stod (val));
            }
            else
            {
                refPathX.push_back (std::stod (val));
            }
            fileValueIndex++;
        }
    }
        
    for (int i=0; i<refPathX.size(); i++)
    {
        std::cout << refPathX[i] << " " << refPathY[i] << "\n";
    }
}