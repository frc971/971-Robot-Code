import {Builder, ByteBuffer} from 'flatbuffers';
import {Configuration} from '../../configuration_generated'
import {BaseType, Configuration as TestTable, FooStruct, Location, Map, VectorOfStrings, VectorOfVectorOfString} from '../../json_to_flatbuffer_generated'

import {Connection} from './proxy';
import {Parser, Table} from './reflection'

// This file runs a basic test to confirm that the typescript flatbuffer
// reflection library is working correctly. It currently is not run
// automatically--to run it, run the web_proxy_demo sh_binary target, open the
// resulting reflection_test.html webpage, open the console and confirm that
// "TEST PASSED" has been printed.

const conn = new Connection();

conn.connect();

function assertEqual(a: any, b: any, msg?: string): void {
  if (a !== b) {
    throw new Error(a + ' !== ' + b + ': ' + msg);
  }
}

// Constructs a flatbuffer and then uses Parser.toObject to parse it and confirm
// that the start/end results are the same. This is largely meant to ensure
// that we are exercising most of the logic associated with parsing flatbuffers.
function DoTest(config: Configuration): void {
  const builder = new Builder();
  {
    TestTable.startVectorFooStructVector(builder, 3);
    const fooStruct0 = FooStruct.createFooStruct(builder, 66, 118);
    const fooStruct1 = FooStruct.createFooStruct(builder, 67, 118);
    const fooStruct2 = FooStruct.createFooStruct(builder, 68, 118);
    const vectorFooStruct = builder.endVector();
    const nameString = builder.createString('nameString');
    const typeString = builder.createString('typeString');
    const location0 =
        Location.createLocation(builder, nameString, typeString, 100, 200);
    const location1 =
        Location.createLocation(builder, nameString, typeString, 300, 400);
    Map.startMap(builder);
    Map.addMatch(builder, location0);
    Map.addRename(builder, location1);
    const map = Map.endMap(builder);

    const mapVector = TestTable.createMapsVector(builder, [map]);

    const strVector =
        VectorOfStrings.createStrVector(builder, [nameString, typeString]);
    const vectorOfStrings =
        VectorOfStrings.createVectorOfStrings(builder, strVector);
    const vVector =
        VectorOfVectorOfString.createVVector(builder, [vectorOfStrings]);
    const vectorOfVectorOfStrings =
        VectorOfVectorOfString.createVectorOfVectorOfString(builder, vVector);

    const doubleVector =
        TestTable.createVectorFooDoubleVector(builder, [9.71, 1.678, 2.056]);

    TestTable.startConfiguration(builder);
    TestTable.addMaps(builder, mapVector);
    TestTable.addVov(builder, vectorOfVectorOfStrings);
    const fooStruct = FooStruct.createFooStruct(builder, 33, 118);
    TestTable.addFooStruct(builder, fooStruct);
    TestTable.addVectorFooStruct(builder, vectorFooStruct);
    TestTable.addVectorFooDouble(builder, doubleVector);
    TestTable.addFooDouble(builder, 11.14);
    TestTable.addFooLong(builder, BigInt('1000000000000'));
    TestTable.addFooEnum(builder, BaseType.Array);
  }

  builder.finish(Configuration.endConfiguration(builder));
  const array = builder.asUint8Array();
  const fbBuffer = new ByteBuffer(array);

  const parsedFb = TestTable.getRootAsConfiguration(fbBuffer);

  let testSchema = null;
  for (let ii = 0; ii < config.channelsLength(); ++ii) {
    if (config.channels(ii).type() === 'aos.testing.Configuration') {
      testSchema = config.channels(ii).schema();
    }
  }
  if (testSchema === null) {
    throw new Error('Couldn\'t find test schema in config.');
  }
  const testParser = new Parser(testSchema);
  const testTable = Table.getRootTable(fbBuffer);
  const testObject = testParser.toObject(testTable, false);

  console.log('Parsed test object:');
  console.log(testObject);

  assertEqual(11.14, parsedFb.fooDouble());
  assertEqual(testObject['foo_double'], parsedFb.fooDouble());
  assertEqual(testObject['foo_enum'], parsedFb.fooEnum());
  assertEqual(testObject['foo_long'], parsedFb.fooLong());
  assertEqual(testObject['foo_ulong'], undefined);
  assertEqual(testObject['locations'], undefined);

  const maps = testObject['maps'];
  assertEqual(maps.length, 1);
  assertEqual(maps[0]['match']['name'], 'nameString');
  assertEqual(maps[0]['rename']['name'], 'nameString');
  assertEqual(maps[0]['match']['type'], 'typeString');
  assertEqual(maps[0]['rename']['type'], 'typeString');
  assertEqual(
      maps[0]['match']['frequency'], parsedFb.maps(0).match().frequency());
  assertEqual(maps[0]['rename']['frequency'], 300);
  assertEqual(maps[0]['match']['max_size'], 200);
  assertEqual(maps[0]['rename']['max_size'], 400);

  assertEqual(
      testObject['foo_struct']['foo_byte'], parsedFb.fooStruct().fooByte());
  assertEqual(
      testObject['foo_struct']['nested_struct']['foo_byte'],
      parsedFb.fooStruct().nestedStruct().fooByte());

  const fooStructs = testObject['vector_foo_struct'];
  assertEqual(fooStructs.length, 3);
  for (let ii = 0; ii < 3; ++ii) {
    assertEqual(
        fooStructs[ii]['foo_byte'], parsedFb.vectorFooStruct(ii).fooByte());
    assertEqual(
        fooStructs[ii]['nested_struct']['foo_byte'],
        parsedFb.vectorFooStruct(ii).nestedStruct().fooByte());
  }

  for (let ii = 0; ii < 3; ++ii) {
    assertEqual(
        testObject['vector_foo_double'][ii], parsedFb.vectorFooDouble(ii));
  }

  assertEqual(testObject['vov']['v'].length, 1);
  assertEqual(testObject['vov']['v'][0]['str'].length, 2);
  assertEqual(testObject['vov']['v'][0]['str'][0], parsedFb.vov().v(0).str(0));
  assertEqual(testObject['vov']['v'][0]['str'][1], parsedFb.vov().v(0).str(1));
  console.log('TEST PASSED');
}

conn.addConfigHandler((config: Configuration) => {
  let configSchema = null;
  for (let ii = 0; ii < config.channelsLength(); ++ii) {
    if (config.channels(ii).type() === 'aos.Configuration') {
      configSchema = config.channels(ii).schema();
    }
  }
  if (configSchema === null) {
    throw new Error('Couldn\'t find Configuration schema in config.');
  }
  let configParser = new Parser(configSchema);
  const configTable = Table.getRootTable(config.bb);
  console.log('Received config:');
  console.log(configParser.toObject(configTable, true));

  DoTest(config);
});
