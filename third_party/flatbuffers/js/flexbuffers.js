"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.flexbuffers = exports.encode = exports.toObject = exports.builder = void 0;
/* eslint-disable @typescript-eslint/no-namespace */
var builder_1 = require("./flexbuffers/builder");
var reference_1 = require("./flexbuffers/reference");
function builder() {
    return new builder_1.Builder();
}
exports.builder = builder;
function toObject(buffer) {
    return reference_1.toReference(buffer).toObject();
}
exports.toObject = toObject;
function encode(object, size, deduplicateStrings, deduplicateKeys, deduplicateKeyVectors) {
    if (size === void 0) { size = 2048; }
    if (deduplicateStrings === void 0) { deduplicateStrings = true; }
    if (deduplicateKeys === void 0) { deduplicateKeys = true; }
    if (deduplicateKeyVectors === void 0) { deduplicateKeyVectors = true; }
    var builder = new builder_1.Builder(size > 0 ? size : 2048, deduplicateStrings, deduplicateKeys, deduplicateKeyVectors);
    builder.add(object);
    return builder.finish();
}
exports.encode = encode;
var builderFunction = builder;
var toObjectFunction = toObject;
var encodeFunction = encode;
var flexbuffers;
(function (flexbuffers) {
    flexbuffers.builder = builderFunction;
    flexbuffers.toObject = toObjectFunction;
    flexbuffers.encode = encodeFunction;
    flexbuffers.toReference = reference_1.toReference;
})(flexbuffers = exports.flexbuffers || (exports.flexbuffers = {}));
exports.default = flexbuffers;
