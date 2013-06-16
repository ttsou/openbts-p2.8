/**@file Common-use GSM declarations, most from the GSM 04.xx and 05.xx series. */
/*
* Copyright 2008-2011 Free Software Foundation, Inc.
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/



#ifndef GSMCOMMON_H
#define GSMCOMMON_H

#include <stdlib.h>
#include <sys/time.h>
#include <ostream>
#include <vector>

#include <Threads.h>
#include <Timeval.h>
#include <BitVector.h>




namespace GSM {

/**@namespace GSM This namespace covers L1 FEC, L2 and L3 message translation. */

/** GSM Training sequences from GSM 05.02 5.2.3. */
extern const BitVector gTrainingSequence[];

/** C0T0 filler burst, GSM 05.02, 5.2.6 */
extern const BitVector gDummyBurst;

/** Random access burst synch. sequence */
extern const BitVector gRACHSynchSequence;


/**@name Modulus operations for frame numbers. */
//@{
/** The GSM hyperframe is largest time period in the GSM system, GSM 05.02 4.3.3. */
const uint32_t gHyperframe = 2048UL * 26UL * 51UL;

/** Get a clock difference, within the modulus, v1-v2. */
int32_t FNDelta(int32_t v1, int32_t v2);

/**
       Compare two frame clock values.
       @return 1 if v1>v2, -1 if v1<v2, 0 if v1==v2
*/
int FNCompare(int32_t v1, int32_t v2);


//@}


/**
	GSM frame clock value. GSM 05.02 4.3
	No internal thread sync.
*/
class Time {

	private:

	int mFN;				///< frame number in the hyperframe
	int mTN;			///< timeslot number

	public:

	Time(int wFN=0, int wTN=0)
		:mFN(wFN),mTN(wTN)
	{ }


	/** Move the time forward to a given position in a given modulus. */
	void rollForward(unsigned wFN, unsigned modulus)
	{
		assert(modulus<gHyperframe);
		while ((mFN % modulus) != wFN) mFN=(mFN+1)%gHyperframe;
	 }

	/**@name Accessors. */
	//@{
	int FN() const { return mFN; }
	void FN(unsigned wFN) { mFN = wFN; }
	unsigned TN() const { return mTN; }
	void TN(unsigned wTN) { mTN=wTN; }
	//@}

	/**@name Arithmetic. */
	//@{

	Time& operator++()
	{
		mFN = (mFN+1) % gHyperframe;
		return *this;
	}

    Time& decTN(unsigned step=1)
    {
		assert(step<=8);
		mTN -= step;
		if (mTN<0) {
			mTN+=8;
			mFN-=1;
			if (mFN<0) mFN+=gHyperframe;
		}
        return *this;
    }

	Time& incTN(unsigned step=1)
	{
		assert(step<=8);
		mTN += step;
		if (mTN>7) {
			mTN-=8;
			mFN = (mFN+1) % gHyperframe;
		}
		return *this;
	}

	Time& operator+=(int step)
	{
		// Remember the step might be negative.
		mFN += step;
		if (mFN<0) mFN+=gHyperframe;
		mFN = mFN % gHyperframe;
		return *this;
	}

	Time operator-(int step) const
		{ return operator+(-step); }

	Time operator+(int step) const
	{
		Time newVal = *this;
		newVal += step;
		return newVal;
	}

	Time operator+(const Time& other) const
    {
        unsigned newTN = (mTN + other.mTN) % 8;
		uint64_t newFN = (mFN+other.mFN + (mTN + other.mTN)/8) % gHyperframe;
        return Time(newFN,newTN);
    } 

	int operator-(const Time& other) const
	{
		return FNDelta(mFN,other.mFN);
	}

	//@}


	/**@name Comparisons. */
	//@{

	bool operator<(const Time& other) const
	{
		if (mFN==other.mFN) return (mTN<other.mTN);
		return FNCompare(mFN,other.mFN)<0;
	}

	bool operator>(const Time& other) const
	{
		if (mFN==other.mFN) return (mTN>other.mTN);
		return FNCompare(mFN,other.mFN)>0;
	}

	bool operator<=(const Time& other) const
	{
		if (mFN==other.mFN) return (mTN<=other.mTN);
		return FNCompare(mFN,other.mFN)<=0;
	}

	bool operator>=(const Time& other) const
	{
		if (mFN==other.mFN) return (mTN>=other.mTN);
		return FNCompare(mFN,other.mFN)>=0;
	}

	bool operator==(const Time& other) const
	{
		return (mFN == other.mFN) && (mTN==other.mTN);
	}

	//@}



	/**@name Standard derivations. */
	//@{

	/** GSM 05.02 3.3.2.2.1 */
	unsigned SFN() const { return mFN / (26*51); }

	/** GSM 05.02 3.3.2.2.1 */
	unsigned T1() const { return SFN() % 2048; }

	/** GSM 05.02 3.3.2.2.1 */
	unsigned T2() const { return mFN % 26; }

	/** GSM 05.02 3.3.2.2.1 */
	unsigned T3() const { return mFN % 51; }

	/** GSM 05.02 3.3.2.2.1. */
	unsigned T3p() const { return (T3()-1)/10; }

	/** GSM 05.02 6.3.1.3. */
	unsigned TC() const { return (FN()/51) % 8; }

	/** GSM 04.08 10.5.2.30. */
	unsigned T1p() const { return SFN() % 32; }

	/** GSM 05.02 6.2.3 */
	unsigned T1R() const { return T1() % 64; }

	//@}
};


std::ostream& operator<<(std::ostream& os, const Time& ts);

}; 	// namespace GSM


#endif

// vim: ts=4 sw=4
