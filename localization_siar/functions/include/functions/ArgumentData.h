#ifndef ARGUMENT_DATA_H_GA
#define ARGUMENT_DATA_H_GA

#include <string>
#include <map>
#include <vector>
#include <stdexcept>
#include <sstream>

namespace functions {
	
	
	
class BadConversion : public std::runtime_error {
public:
	BadConversion(const std::string& s): std::runtime_error(s) { }
};

class ArgumentData:public std::vector<std::string> {
	public: 
		
	//! @brief Default constructor
	ArgumentData();
	
	//! @brief Recommended constructor
	//! @param argc Argument number (from main)
	//! @param argv Argument values (from main)
	ArgumentData(int argc, char **argv);
	
	//! @brief Initializes all fields of the class
	void init();
	
	
	//! @brief Returns a string that represent the contents of the class
	//! @return The string with the desired contents
	std::string toString() const;
	
	//! @brief Gets the parameters associated with and option.
	//! @param op Option name.
	//! @param op_param Parameters associated with the option.
	//! @retval true The option was found.
	//! @retval false The option was not found.
	bool getOption(const std::string& op, std::vector< std::string >& op_param) const;
	
	//! @brief Returns true if the options is included in the arguments.
	//! @param op Option name.
	//! @retval true The option was found.
	//! @retval false The option was not found.
	inline bool isOption(const std::string &op);
	
	//! @brief Get the option with one parameter in a determinate format.
	//! @param op Option name.
	//! @retval true The option was found.
	//! @retval false The option was not found.
	template<typename T> bool getOption(const std::string &op, T &value) const;
	
	//! @brief Get the option with one parameter in a determinate format.
	//! @param op Parameter position.
	//! @retval true The parameter was found.
	//! @retval false The parameter was not found.
	template<typename T> bool getParameter(int num, T &value) const;
	
	private:
	//! Option map relates option names with their parameters
	std::map <std::string, std::vector<std::string> > options;
   
	//! Interprets the arguments storing the options into the map
	void interpretArguments();
};

template<typename T> bool ArgumentData::getOption(const std::string& op, T& value) const
{
	std::map<std::string, std::vector<std::string> >::const_iterator it = options.find(op);
	bool ret_val = false;
	
	if ( it != options.end() && it->second.size() == 1 ) {
		ret_val = true;
		char c;
		
		std::istringstream i( it->second[0] );
		if ( !(i >> value) || (i.get(c))) {
			throw BadConversion( it->second[0] );
		}
	}
	
	return ret_val;
}

template<typename T> bool ArgumentData::getParameter(int num, T& value) const
{
	bool ret_val = false;
	
	if (num < size()) {
	  ret_val = true;
	  char c;
	  std::istringstream i( at(num) );
		if ( !(i >> value) || (i.get(c))) {
			throw BadConversion( at(num) );
		}
	}
	
	return ret_val;
}

inline bool ArgumentData::isOption(const std::string& op)
{
	return options.find(op) != options.end();
}


}

#endif