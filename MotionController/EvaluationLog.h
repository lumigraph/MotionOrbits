#pragma once

#include <string>

struct Evaluation
{

};

class EvaluationLog
{
public:
	EvaluationLog();
	virtual ~EvaluationLog();

	bool load( std::string& path );
	bool save( std::string& path );

protected:

};
