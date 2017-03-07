#include "ArgumentData.h"

#include <sstream>
#include <iostream>

using namespace std;

namespace functions {

ArgumentData::ArgumentData()
{
	init();
}


ArgumentData::ArgumentData(int argc, char** argv)
{
	init();
	
	// Copy the contents of the argv to the class
	for (int i = 0; i < argc; i++) {
		this->push_back(string(argv[i]));
	}
	
	interpretArguments(); // interprets the arguments and saves the results into a map
}

std::string ArgumentData::toString() const
{
	ostringstream oss;

	for (unsigned int i = 0; i < this->size(); i++) {
		oss << (*this)[i] << " ";
	}
	
// 	oss << endl;
	
	return oss.str();
}

void ArgumentData::init()
{
	this->clear();
	options.clear();
}

void ArgumentData::interpretArguments()
{
	for (unsigned int i = 1; i < size(); i++) {
		string &curr = (*this)[i];
		if ( curr.size() > 3 && curr.at(0) == '-' && (*this)[i].at(1) == '-') {
			string name(curr.substr(2, curr.size()));
			
			std::vector<string> curr_params;
			bool ended = false;
			
			i++;
			for (  ; i < size() && !ended; i++) {
				string &curr = (*this)[i];
				
				if ( curr.size() > 3 && curr.at(0) == '-' && (*this)[i].at(1) == '-') {
					ended = true;
					i -= 2; // Go back and consider this option.
				} else {
					curr_params.push_back(curr);
				}
				
			}
			
			options.insert(pair<string, vector<string> >(name, curr_params));
		}
	}
}


bool ArgumentData::getOption(const std::string& op, vector< string > &op_param) const
{
	bool ret_val = false;
	
	map<std::string, vector<string> >::const_iterator it = options.find(op);
	
	if ( it != options.end() ) {
		ret_val = true;
		
		op_param = it->second;
	}
	
	return ret_val;
}



}
