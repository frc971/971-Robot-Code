/*
 * ArrayEntryType.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: Mitchell Wills
 */

#include "networktables2/type/ArrayEntryType.h"

ArrayEntryType::ArrayEntryType(TypeId id, NetworkTableEntryType& elementType)
	: ComplexEntryType(id, "Array"), m_elementType(elementType){ //TODO super(id, "Array of [" + elementType.name + "]");
}

EntryValue ArrayEntryType::copyElement(EntryValue value){
  return m_elementType.copyValue(value);
}
void ArrayEntryType::deleteElement(EntryValue value){
  m_elementType.deleteValue(value);
}


void ArrayEntryType::sendValue(EntryValue value, DataIOStream& os) {
	ArrayEntryData* dataArray = (ArrayEntryData*) value.ptr;
	/*if (dataArray->length > 255) {//TODO throw better exception
		throw IOException("Cannot write " + value + " as " + name + ". Arrays have a max length of 255 values");
	}*/
	os.writeByte(dataArray->length);
	for (int i = 0; i < dataArray->length; ++i) {
		m_elementType.sendValue(dataArray->array[i], os);
	}
}

EntryValue ArrayEntryType::readValue(DataIOStream& is) {
	uint8_t length = is.readByte();
	EntryValue* array = (EntryValue*)malloc(length*sizeof(EntryValue));//TODO cache object arrays
	for (int i = 0; i < length; ++i) {
		array[i] = m_elementType.readValue(is);
	}
	
	ArrayEntryData* dataArray = (ArrayEntryData*)malloc(sizeof(ArrayEntryData));
	dataArray->length = length;
	dataArray->array = array;
	
	EntryValue eValue;
	eValue.ptr = dataArray;
	return eValue;
}

EntryValue ArrayEntryType::copyValue(EntryValue value){
	ArrayEntryData* otherDataArray = (ArrayEntryData*) value.ptr;

	EntryValue* array = (EntryValue*)malloc(otherDataArray->length*sizeof(EntryValue));//TODO cache object arrays
	for (int i = 0; i < otherDataArray->length; ++i) {
		array[i] = m_elementType.copyValue(otherDataArray->array[i]);
	}
	
	ArrayEntryData* dataArray = (ArrayEntryData*)malloc(sizeof(ArrayEntryData));
	dataArray->length = otherDataArray->length;
	dataArray->array = array;
	
	EntryValue eValue;
	eValue.ptr = dataArray;
	return eValue;
}
void ArrayEntryType::deleteValue(EntryValue value){
	ArrayEntryData* dataArray = (ArrayEntryData*) value.ptr;
	if(dataArray!=NULL){
	  for (int i = 0; i < dataArray->length; ++i) {
	    m_elementType.deleteValue(dataArray->array[i]);
	  }
	  free(dataArray->array);
	  free(dataArray);
	}
}

EntryValue ArrayEntryType::internalizeValue(std::string& key, ComplexData& externalRepresentation, EntryValue currentInteralValue) {
	// TODO: Argument 'key' appears unused.
	ArrayData& externalArrayData = (ArrayData&)externalRepresentation;
	ArrayEntryData* internalArray = (ArrayEntryData*) currentInteralValue.ptr;
	if(internalArray != NULL && internalArray->length==externalArrayData.size()){
		for(int i = 0; i<internalArray->length; ++i){
			m_elementType.deleteValue(internalArray->array[i]);
			internalArray->array[i] = m_elementType.copyValue(externalArrayData._get(i));
		}
		return currentInteralValue;
	}
	else{
		internalArray = (ArrayEntryData*)malloc(sizeof(ArrayEntryData));
		internalArray->array = (EntryValue*)malloc(externalArrayData.size()*sizeof(EntryValue));//TODO cache object arrays
		internalArray->length = externalArrayData.size();
		for (int i = 0; i < internalArray->length; ++i) {
			internalArray->array[i] = m_elementType.copyValue(externalArrayData._get(i));
		}
		EntryValue eValue;
		eValue.ptr = internalArray;
		return eValue;
	}
}

void ArrayEntryType::exportValue(std::string& key, EntryValue internalData, ComplexData& externalRepresentation) {
	ArrayEntryData* internalArray = (ArrayEntryData*) internalData.ptr;
	ArrayData& externalArrayData = (ArrayData&)externalRepresentation;
	externalArrayData.setSize(internalArray->length);
	for(int i = 0; i<internalArray->length; ++i){
		externalArrayData._set(i, m_elementType.copyValue(internalArray->array[i]));
	}
}
