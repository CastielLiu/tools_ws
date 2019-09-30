#include<iostream>
#include<fstream>
#include<boost/filesystem.hpp>

namespace fs = boost::filesystem;
using namespace std;

//g++ file_batch_processing.cpp -std=c++11 -lboost_system -lboost_filesystem

void modifyFile(string& file_name)
{
	ifstream infile;
	infile.open(file_name.c_str());
	if(!infile.is_open())
	{
		cout << "open "<< file_name << " failed!!";
		return;
	}
	
	string file_path = file_name.substr(0,file_name.find_last_of("/"));
	string temp_file_name = file_path+"/temp.xml";
	
	ofstream outfile;
	outfile.open(temp_file_name.c_str());
	
	string line;
	
	bool maintainer_ok = false, author_ok = false;
	
	while(!infile.eof())
	{
		getline(infile,line);
		if(line.find("maintainer email=\"") < line.length())
		{
			if(!maintainer_ok)
			{
				outfile << "  <maintainer email=\"castiel_liu@outlook.com\">wendao</maintainer>"<< endl;
				maintainer_ok = true;
			}
		}
		else if(line.find("<author email=\"") < line.length())
		{
			if(!author_ok)
			{
				outfile << "  <author email=\"castiel_liu@outlook.com\">wendao</author>" << endl;
				author_ok = true;
			}
		}
		else
			outfile << line << endl;
	}
	
	fs::remove(fs::path(file_name));
	fs::rename(fs::path(temp_file_name), fs::path(file_name));
	
	cout << file_name << " modify ok" << endl;

}

int main(int argc,char**argv)
{
	if(argc<2)
	{
		cout << "please input the initial path ^o^ " << endl;
		return 0;
	}
	
	fs::recursive_directory_iterator begin(argv[1]);
	fs::recursive_directory_iterator end;
	for(auto iter=begin; iter!=end; ++iter)
	{
		if(fs::is_directory(*iter))
			continue;
		//auto file = fs::system_complete(*iter).leaf().string();
		string file_name = fs::system_complete(*iter).string();
		if(fs::extension(file_name) != ".xml")
			continue;
		modifyFile(file_name);
	}
	return 0;
}

