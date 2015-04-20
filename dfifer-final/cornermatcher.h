#ifndef _CORNERMATCHER_H_
#define _CORNERMATCHER_H_

#include "chessboard.h"

class CornerMatcher {
 public:
	CornerMatcher(const Chessboard&, const Chessboard&);
	void match();
 private:
	Chessboard _cb1;
	Chessboard _cb2;
};

#endif // _CORNERMATCHER_H_
