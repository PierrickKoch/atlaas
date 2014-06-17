/*
 * io.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2014-03-19
 * license: BSD
 */
#ifndef ATLAAS_IO_HPP
#define ATLAAS_IO_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <cassert>
#include <atlaas/common.hpp>

using namespace std;
namespace atlaas {

/*
 * Dump a point cloud and its transformation matrix 
 * into a file <filename>
 */
inline void dump(const std::string& filepath,
	const points& cloud, const matrix& transformation) {

	ofstream out_file;

	out_file.open(filepath, ios::out | ios::trunc | ios::binary);
	if(out_file.fail())
	{
		cout << "\nCannot open file " << filepath << "for dumping data \n";
		system("pause");
		return;
	}

	try{
		out_file.write((char *)&transformation[0], sizeof(matrix));
		int size = cloud.size();
		out_file.write((char*)&size,sizeof(int));
		out_file.write((char *)&cloud[0],sizeof(point_xy_t)*cloud.size());
		// cout << "number of bytes wrote " << out_file.gcount(); 
	} catch (exception &e){
		cout << "wrting error " << e.what() << endl;
	}
	cout << "write cloud data into " << filepath << "done \n";
	out_file.close();
	
}

inline void load(const std::string& filepath, points& cloud, matrix& transformation) {
    ifstream in_file;
   	in_file.open(filepath, ios::in | ios::binary);
	if(in_file.fail())
	{	
		cout << "\nCannot open file " << filepath << "for reading data \n";
		system("pause");
		return;
	}


    // Read header
    in_file.read((char*)&transformation[0],sizeof(matrix));

		// read Number of point 
	try{
		int n_point = -1;
		in_file.read((char*)&n_point,sizeof(int));
		assert(n_point >=0);
		cloud.resize(n_point);
		in_file.read((char*)&cloud[0],n_point*sizeof(point_xy_t));
	} catch (exception& e){
		cout << "read error " << e.what() << endl;
	}

	in_file.close();
}

// Print the transformation matrix
inline void print_transformation(const matrix &transform){
    std::cout << "-------------------------------\n";
    std::cout << "Transformation Matrix " << std::endl;

    for(uint i = 0; i < transform.size(); i++){
        cout << transform[i] << "\t\t";
        if(i%4 == 3) std::cout << std::endl;
    }
    std::cout << "-------------------------------\n";

}

inline void print_6dPose(const pose6d& pose){

    std::cout << "euler angles : yaw = " << pose[0] << " pitch = " << pose[1] << " roll= " << pose[2] << std::endl;
    std::cout << "Coord        :   x = " << pose[3] << "     y = " << pose[4] << "    z= " << pose[5] << std::endl;
}


} // namespace atlaas

#endif // ATLAAS_IO_HPP

