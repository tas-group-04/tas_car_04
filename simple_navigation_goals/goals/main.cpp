#include <QCoreApplication>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <qstring.h>
#include <qstringlist.h>

using namespace std;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    ifstream file;
    file.open("../../goals.txt");

    string s_line;
    QString line;

    int num=0;

    bool* ok;

    double px,py,pz,ox,oy,oz,ow;

    while(!file.eof()){
        getline(file,s_line);
        line = s_line.c_str();
        if(line.contains("position")){
            num++;
            getline(file,s_line);
            line = s_line.c_str();
            QStringList value;
            value = line.split(":", QString::SkipEmptyParts);
            px = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            py = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            pz = value.at(1).toDouble(ok);
            getline(file,s_line);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            ox = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            oy = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            oz = value.at(1).toDouble(ok);
            getline(file,s_line);
            line = s_line.c_str();
            value = line.split(":", QString::SkipEmptyParts);
            ow = value.at(1).toDouble(ok);


            cout << " Point " << num << ": " << endl;
            cout << "\tx: " << px << "   y: " << py << "   z: " << pz << endl;
            cout << "\to_x: " << ox << "   o_y: " << oy << "   o_z: " << oz << "   o_w: " << ow << endl;
        }
    }

    return a.exec();
}
