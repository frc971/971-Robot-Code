"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    Object.defineProperty(o, k2, { enumerable: true, get: function() { return m[k]; } });
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.flatbuffers = void 0;
/* eslint-disable @typescript-eslint/no-namespace */
var constants = __importStar(require("./constants"));
var utils = __importStar(require("./utils"));
var long_1 = require("./long");
var encoding_1 = require("./encoding");
var builder_1 = require("./builder");
var byte_buffer_1 = require("./byte-buffer");
var flatbuffers;
(function (flatbuffers) {
    flatbuffers.SIZEOF_SHORT = constants.SIZEOF_SHORT;
    flatbuffers.SIZEOF_INT = constants.SIZEOF_INT;
    flatbuffers.FILE_IDENTIFIER_LENGTH = constants.FILE_IDENTIFIER_LENGTH;
    flatbuffers.SIZE_PREFIX_LENGTH = constants.SIZE_PREFIX_LENGTH;
    flatbuffers.Encoding = encoding_1.Encoding;
    flatbuffers.int32 = utils.int32;
    flatbuffers.float32 = utils.float32;
    flatbuffers.float64 = utils.float64;
    flatbuffers.isLittleEndian = utils.isLittleEndian;
    flatbuffers.Long = long_1.Long;
    flatbuffers.Builder = builder_1.Builder;
    flatbuffers.ByteBuffer = byte_buffer_1.ByteBuffer;
})(flatbuffers = exports.flatbuffers || (exports.flatbuffers = {}));
exports.default = flatbuffers;
