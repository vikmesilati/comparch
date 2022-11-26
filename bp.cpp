/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"

#include<iostream>
#include <cmath>
#include <vector>
using namespace std;

// Enums //////////////////////////////////////////////////////
enum CounterState
{
	SNT = 0,
	WNT = 1,
	WT = 2,
	ST = 3
};

enum ShareOptions
{
	not_using_share = 0,
	using_share_lsb = 1,
	using_share_mid = 2,
};

enum PredictorMode
{
	local_table_local_history = 0,
	local_table_global_history = 1,
	global_table_local_history = 2,
	global_table_global_history = 3
};
/////////////////////////////////////////////////////////////////

// Structs //////////////////////////////////////////////////////
SIM_stats* InitSimStats(unsigned flush_num, unsigned br_num, unsigned size)
{
	SIM_stats* s = new SIM_stats();
	s->flush_num = flush_num;
	s->br_num = br_num;
	// Theoretical predictor size in bits(including valid bits).
	s->size = size;
	return s;
}

SIM_stats* GlobalStats = InitSimStats(0, 0, 0);

/////////////////////////////////////////////////////////////////

// Classes //////////////////////////////////////////////////////

class BimodalCounter
{
	private:
		CounterState _curState;

	public:
		// Dummy constructor.
		BimodalCounter() {};

		BimodalCounter(CounterState initialFsmState)
		{
			_curState = initialFsmState;
		}

		void SetNextCounterState(bool isTaken)
		{
			switch (_curState)
			{
				case SNT:
					if (isTaken == true)
					{
						_curState = WNT;
					}
					break;
				case WNT:
					if (isTaken == true)
					{
						_curState = WT;
					}
					else
					{
						_curState = SNT;
					}
					break;
				case WT:
					if (isTaken == true)
					{
						_curState = ST;
					}
					else
					{
						_curState = WNT;
					}
					break;
				case ST:
					if (isTaken == false)
					{
						_curState = WT;
					}
					break;
			}
		}

		bool IsTaken()
		{
			switch (_curState)
			{
				case SNT:
				case WNT:
					return false;
				case WT:
				case ST:
					return true;
			}
		}
};

class BTBEntry
{
	private:
		bool _isInValidEntry;
		uint32_t _tag;
		// BHR Register - per branch.
		uint32_t _bhr;
		bool _isTaken;
		uint32_t _targetPc;
		// For local FSMs.
		vector<BimodalCounter> _lCounterArray; 

		// GHR Register - for all the branches.
		static uint32_t _ghr;
		// For global FSMs.
		static vector<BimodalCounter> _gCounterArray; 	

		static unsigned _historySize;
		static unsigned _fsmState;
		static PredictorMode _predictorMode;
		static int _shared;

	public:
		// Dummy constructor.
		BTBEntry() 
		{
			_isInValidEntry = true;
		}
		~BTBEntry(){
			_lCounterArray.clear();
			_gCounterArray.clear();
		}
		BTBEntry(uint32_t tag, uint32_t targetPc, bool isTaken)
		{
			_isInValidEntry = false;
			_tag = tag;
			// History for new branch is 0.
			_bhr = 0;
			_targetPc = targetPc;
			_isTaken = isTaken;

			// If the entery isn't in the BTB, 
			// the predicition is NT.
			if (isTaken == true)
			{
				GlobalStats->flush_num++;
			}

			switch (_predictorMode)
			{
				case local_table_local_history:
					_lCounterArray.resize(pow(2, _historySize));
					InitLocalFSMs();
					_lCounterArray[_bhr].SetNextCounterState(isTaken);
					UpdateHistory(_bhr, isTaken);
					break;
				case local_table_global_history:
					_lCounterArray.resize(pow(2, _historySize));
					InitLocalFSMs();
					_lCounterArray[_ghr].SetNextCounterState(isTaken);
					UpdateHistory(_ghr, isTaken);
					break;
				case global_table_local_history:
					_gCounterArray[_bhr].SetNextCounterState(isTaken);
					UpdateHistory(_bhr, isTaken);
					break;
				case global_table_global_history:
					_gCounterArray[_ghr].SetNextCounterState(isTaken);
					UpdateHistory(_ghr, isTaken);
					break;
			}
		}

		void InitLocalFSMs()
		{
			for (int i = 0; i < _lCounterArray.size(); i++)
			{
				_lCounterArray[i] = BimodalCounter((CounterState)_fsmState);
			}
		}

		static void InitGlobalFSMs()
		{
			for (int i = 0; i < _gCounterArray.size(); i++)
			{
				_gCounterArray[i] = BimodalCounter((CounterState)_fsmState);
			}
		}

		static void BTBEntrySetParams(unsigned historySize, unsigned fsmState, bool isGlobalHist, 
									  bool isGlobalTable, int shared, PredictorMode predictMode)
		{
			_historySize = historySize;
			_fsmState = fsmState;
			_predictorMode = predictMode;
			_shared = shared;

			if (isGlobalTable)
			{
				_gCounterArray.resize(pow(2, historySize));
				InitGlobalFSMs();
			}
		}

		bool IsInValidEntry()
		{
			return _isInValidEntry;
		}

		uint32_t GetEntryTag()
		{
			return _tag;
		}

		bool GetPredictedPCDst(uint32_t pc, uint32_t* dst)
		{
			bool isBranchTaken;
			uint32_t hRegisterWithShare;

			switch (_predictorMode)
			{
				case local_table_local_history:
					isBranchTaken = _lCounterArray[_bhr].IsTaken();
					*dst = isBranchTaken ? _targetPc : pc + 4;
					break;
				case local_table_global_history:
					isBranchTaken = _lCounterArray[_ghr].IsTaken();
					*dst = isBranchTaken ? _targetPc : pc + 4;
					break;
				case global_table_local_history:
					hRegisterWithShare = GetHistoryRegisterAfterXOR(_bhr, pc);
					isBranchTaken = _gCounterArray[hRegisterWithShare].IsTaken();
					*dst = isBranchTaken ? _targetPc : pc + 4;
					break;
				case global_table_global_history:
					hRegisterWithShare = GetHistoryRegisterAfterXOR(_ghr, pc);
					isBranchTaken = _gCounterArray[hRegisterWithShare].IsTaken();
					*dst = isBranchTaken ? _targetPc : pc + 4;
					break;
			}

			return isBranchTaken;
		}

		void UpdateBTBEntry(uint32_t pc, uint32_t targetPc, bool taken)
		{
			_targetPc = targetPc;

			uint32_t tmpDst = 0;

			if (GetPredictedPCDst(pc, &tmpDst) != taken)
			{
				GlobalStats->flush_num++;
			}

			uint32_t hRegisterWithShare;

			switch (_predictorMode)
			{
				case local_table_local_history:
					_lCounterArray[_bhr].SetNextCounterState(taken);
					UpdateHistory(_bhr, taken);
					break;
				case local_table_global_history:
					_lCounterArray[_ghr].SetNextCounterState(taken);
					UpdateHistory(_ghr, taken);
					break;
				case global_table_local_history:
					hRegisterWithShare = GetHistoryRegisterAfterXOR(_bhr, pc);
					_gCounterArray[hRegisterWithShare].SetNextCounterState(taken);
					UpdateHistory(_bhr, taken);
					break;
				case global_table_global_history:
					hRegisterWithShare = GetHistoryRegisterAfterXOR(_ghr, pc);
					_gCounterArray[hRegisterWithShare].SetNextCounterState(taken);
					UpdateHistory(_ghr, taken);
					break;
			}
		}

		// Getting the result of XOR between pc mask
		// and history value.
		uint32_t GetHistoryRegisterAfterXOR(uint32_t& historyRegister, uint32_t pc)
		{
			bool isShareLsb = false;
			uint32_t pcMask;
			uint32_t hRegisterWithShare;

			switch (_shared)
			{
				case not_using_share:
					hRegisterWithShare = historyRegister;
					break;
				case using_share_lsb:
					isShareLsb = true;
					pcMask = GetPCMaskWithShare(pc, isShareLsb);
					hRegisterWithShare = historyRegister ^ pcMask;
					break;
				case using_share_mid:
					pcMask = GetPCMaskWithShare(pc, isShareLsb);
					hRegisterWithShare = historyRegister ^ pcMask;				
					break;
			}

			return hRegisterWithShare;
		}

		// Relevant to LShare/GShare - building pc mask to do XOR
		// with history value.
		uint32_t GetPCMaskWithShare(uint32_t pc, bool isShareLsb)
		{
			uint32_t numOfBitsShiftRight = isShareLsb ? 2 : 16;

			uint32_t pcMask = 0;

			for (int i = 0; i < _historySize; i++)
			{
				// Inserting 1 bit from the right.
				pcMask = (pcMask << 1) | 1;
			}

			pcMask = (pc >> numOfBitsShiftRight) & pcMask;

			return pcMask;
		}

		void UpdateHistory(uint32_t&historyRegister, bool taken)
		{
			unsigned newBit = taken ? 1 : 0;
			unsigned historyModulo = pow(2, _historySize);
			historyRegister = ((historyRegister << 1) | newBit) % historyModulo;
		}
};

unsigned BTBEntry::_historySize;
unsigned BTBEntry::_fsmState;
PredictorMode BTBEntry::_predictorMode;
int BTBEntry::_shared;
uint32_t BTBEntry::_ghr = 0;
vector<BimodalCounter> BTBEntry::_gCounterArray;

class BranchPredictor
{
	private:
		vector<BTBEntry> _btbEntries;
		static unsigned _btbSize;
		static unsigned _tagSize;
		static unsigned _historySize;

	public:
		BranchPredictor() {};
		BranchPredictor(unsigned btbSize) 
		{
			_btbEntries.resize(btbSize);
		}
		~BranchPredictor(){
			vector<BTBEntry>().swap(_btbEntries);
			_btbEntries.clear();
		}
		int Init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int shared)
		{
			// Return -1 = init failure.
			if (btbSize < 0) return -1;
			_btbSize = btbSize;

			if (historySize < 0) return -1;
			_historySize = historySize;

			if (tagSize < 0) return -1;
			_tagSize = tagSize;

			if (fsmState < SNT || fsmState > ST) return -1;

			if (shared < not_using_share || shared > using_share_mid) return -1;

			PredictorMode predictMode = FigurePredictorMode(isGlobalTable, isGlobalHist);

			// Initis
			BTBEntry::BTBEntrySetParams(historySize, fsmState, isGlobalHist, isGlobalTable, shared, predictMode);
		
			// Success
			return 0;
		}

		PredictorMode FigurePredictorMode(bool isGlobalTable, bool isGlobalHist)
		{
			unsigned validBitSize = 1;
			unsigned targetPcSize = 30;

			if (isGlobalTable && isGlobalHist)
			{
				GlobalStats->size = _btbSize * (validBitSize + targetPcSize + _tagSize) + _historySize + 2 * pow(2, _historySize);
				return global_table_global_history;
			}
			// Shared counter arrays
			else if (isGlobalTable && (isGlobalHist == false))
			{
				GlobalStats->size = _btbSize * (validBitSize + targetPcSize + _tagSize + _historySize) + 2 * pow(2, _historySize);
				return global_table_local_history;
			}
			else if ((isGlobalTable == false) && isGlobalHist)
			{
				GlobalStats->size = _btbSize * (validBitSize + targetPcSize + _tagSize + 2 * pow(2, _historySize)) + _historySize;
				return local_table_global_history;
			}
			// Private counter arrays.
			else
			{
				GlobalStats->size = _btbSize * (validBitSize + targetPcSize + _tagSize + _historySize + 2 * pow(2, _historySize));
				return local_table_local_history;
			}
		}

		bool Predict(uint32_t pc, uint32_t* dst)
		{
			// In case in BTB.
			if (IsInBTB(pc))
			{
				uint32_t btbEntryIndex = GetBTBEntryIndex(pc);
				return _btbEntries[btbEntryIndex].GetPredictedPCDst(pc, dst);
			}
			else
			{
				*dst = pc + 4;
				return false;
			}
		}

		void UpdateBTB(uint32_t pc, uint32_t targetPc, bool taken)
		{
			// First, we need to check if the branch is in the BTB.

			// In case in BTB.
			if (IsInBTB(pc))
			{
				uint32_t btbEntryIndex = GetBTBEntryIndex(pc);
				_btbEntries[btbEntryIndex].UpdateBTBEntry(pc, targetPc, taken);
			}
			// In case not in BTB.
			else
			{
				AddToBTB(pc, targetPc, taken);
			}

			GlobalStats->br_num++;
		}

		bool IsInBTB(uint32_t pc)
		{
			uint32_t btbEntryIndex = GetBTBEntryIndex(pc);

			if (_btbEntries[btbEntryIndex].IsInValidEntry())
			{
				return false;
			}

			uint32_t pcTag = CalcTag(pc);

			if (pcTag == _btbEntries[btbEntryIndex].GetEntryTag())
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		void AddToBTB(uint32_t pc, uint32_t targetPc, bool taken)
		{
			// Here we add a new line to the Branch predictor.
			int btbEntryIndex = GetBTBEntryIndex(pc);

			_btbEntries[btbEntryIndex] = BTBEntry(CalcTag(pc), GetAlignedPCTarget(targetPc), taken);
		}

		uint32_t GetBTBEntryIndex(uint32_t pc)
		{
			uint32_t numOfBitsShiftRight = 2;

			uint32_t pcMask = 0;

			for (int i = 0; i < _btbSize; i++)
			{
				// Inserting 1 bit from the right.
				pcMask = (pcMask << 1) | 1;
			}

			uint32_t btbEntryIndex = ((pc >> numOfBitsShiftRight) & pcMask) % _btbSize;

			return btbEntryIndex;
		}

		uint32_t CalcTag(uint32_t pc)
		{
			uint32_t numOfBitsShiftRight = 2 + log2(_btbSize);

			uint32_t pcMask = 0;

			for (int i = 0; i < _tagSize; i++)
			{
				// Inserting 1 bit from the right.
				pcMask = (pcMask << 1) | 1;
			}

			uint32_t tag = (pc >> numOfBitsShiftRight) & pcMask;

			return tag;
		}

		uint32_t GetAlignedPCTarget(uint32_t targetPc)
		{
			uint32_t numOfBitsShiftRight = 2;

			uint32_t pcMask = 0xFFFFFFFC;

			uint32_t alignedTargetPC = targetPc & pcMask;

			return alignedTargetPC;
		}
};

unsigned BranchPredictor::_btbSize;
unsigned BranchPredictor::_tagSize = 0;
unsigned BranchPredictor::_historySize = 0;

// Globals //////////////////////////////////////////////////////
// 
// Global object of BranchPredictor class in oder
// to use inside the BP functions which isn't part of the class.
BranchPredictor* bp;
/////////////////////////////////////////////////////////////////

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
	bool isGlobalHist, bool isGlobalTable, int Shared)
{
	bp = new BranchPredictor(btbSize);

	return bp->Init(btbSize, historySize, tagSize, fsmState,
		isGlobalHist, isGlobalTable, Shared);
}

bool BP_predict(uint32_t pc, uint32_t* dst) {
	return bp->Predict(pc, dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
	// pred_dst is not relevant to update, just for predict.
	bp->UpdateBTB(pc, targetPc, taken);
}

void BP_GetStats(SIM_stats* curStats) {
	curStats->br_num = GlobalStats->br_num;
	curStats->flush_num = GlobalStats->flush_num;
	curStats->size = GlobalStats->size;

	// Releasing dynamic allocations.
	delete GlobalStats;
	// Release bp and entries.
	delete bp;
	return;
}