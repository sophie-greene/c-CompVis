#include <QtCore/QCoreApplication>
#include <iostream>
#include <cstdlib>
#include <vector>

using namespace std;
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);




        vector< vector<int> > buff(10);

        for(int i=0; i<10; i++){
                int y = rand()%10; //gen random num to test operability
                buff[y][0].push_back(1);
        }

        for(int i=0; i<10; i++)
                for(int p=0; p<10; p++)
                cout << buff[i][p] << " ";

    return a.exec();
}
