
#ifndef XDAUTILS_H
#define XDAUTILS_H

#include <xstypes/xsdataidentifier.h>
#include <string>

std::string get_xs_data_identifier_name(XsDataIdentifier identifier);
bool get_xs_data_identifier_by_name(std::string name, XsDataIdentifier& identifier);
bool parseConfigLine(std::string line, std::string& name, int& value);

#endif
