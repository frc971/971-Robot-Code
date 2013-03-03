/*
 * DefaultEntryTypes.h
 *
 *  Created on: Sep 24, 2012
 *      Author: Mitchell Wills
 */

#ifndef DEFAULTENTRYTYPES_H_
#define DEFAULTENTRYTYPES_H_

class DefaultEntryTypes;

#include "NetworkTableEntryTypeManager.h"



static const TypeId BOOLEAN_RAW_ID = 0x00;
static const TypeId DOUBLE_RAW_ID = 0x01;
static const TypeId STRING_RAW_ID = 0x02;

class DefaultEntryTypes{
private:
	/**
	 * a boolean entry type
	 */
	class BOOLEAN_t : public NetworkTableEntryType{
	public:
		BOOLEAN_t();
		/**
		 * See {@link NetworkTableEntryType}::sendValue
		 */
		virtual void sendValue(EntryValue value, DataIOStream& os);
		
		/**
		 * See {@link NetworkTableEntryType}::readValue
		 */
		virtual EntryValue readValue(DataIOStream& is);
	};
	/**
	 * a double floating point entry type
	 */
	class DOUBLE_t : public NetworkTableEntryType{
	public:
		DOUBLE_t();
		
		/**
		 * See {@link NetworkTableEntryType}::sendValue
		 */
		virtual void sendValue(EntryValue value, DataIOStream& os);
		
		/**
		 * See {@link NetworkTableEntryType}::sendValue
		 */
		virtual EntryValue readValue(DataIOStream& is);
	};
	/**
	 * a string entry type
	 */
	class STRING_t : public NetworkTableEntryType{
	public:
		STRING_t();
		
		/**
		 * See {@link NetworkTableEntryType}::sendValue
		 */
		virtual void sendValue(EntryValue value, DataIOStream& os);
		
		/**
		 * See {@link NetworkTableEntryType}::sendValue
		 */
		virtual EntryValue readValue(DataIOStream& is);
		
		/**
		 * See {@link NetworkTableEntryType}::copyValue
		 */
		virtual EntryValue copyValue(EntryValue value);
		
		/**
		 * See {@link NetworkTableEntryType}::deleteValue
		 */
		virtual void deleteValue(EntryValue value);
	};
public:
	/**
	 * Registers the set of default entry types with the given 
	 * entry type manager.
	 * 
	 * @param typeManager The entry type manager to update. Pointer must not be null.
	 */
	static void registerTypes(NetworkTableEntryTypeManager* typeManager);
	
	static BOOLEAN_t BOOLEAN;
	static DOUBLE_t DOUBLE;
	static STRING_t STRING;
};




#endif /* DEFAULTENTRYTYPES_H_ */
