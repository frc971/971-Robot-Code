#include "aos/util/mcap_logger.h"

#include <iostream>

#include "flatbuffers/reflection_generated.h"
#include "gtest/gtest.h"

namespace aos::testing {
// TODO(james): Write a proper test for the McapLogger itself. However, that
// will require writing an MCAP reader (or importing an existing one).

// Confirm that the schema for the reflection.Schema table itself hasn't
// changed. reflection.Schema should be a very stable type, so this should
// need
// updating except when we change the JSON schema generation itself.
TEST(JsonSchemaTest, ReflectionSchema) {
  std::string schema_json =
      JsonSchemaForFlatbuffer({reflection::Schema::MiniReflectTypeTable()})
          .dump(4);
  EXPECT_EQ(R"json({
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "properties": {
        "advanced_features": {
            "type": "number"
        },
        "enums": {
            "items": {
                "properties": {
                    "attributes": {
                        "items": {
                            "properties": {
                                "key": {
                                    "type": "string"
                                },
                                "value": {
                                    "type": "string"
                                }
                            },
                            "type": "object"
                        },
                        "type": "array"
                    },
                    "declaration_file": {
                        "type": "string"
                    },
                    "documentation": {
                        "items": {
                            "type": "string"
                        },
                        "type": "array"
                    },
                    "is_union": {
                        "type": "boolean"
                    },
                    "name": {
                        "type": "string"
                    },
                    "underlying_type": {
                        "properties": {
                            "base_size": {
                                "type": "number"
                            },
                            "base_type": {
                                "type": "number"
                            },
                            "element": {
                                "type": "number"
                            },
                            "element_size": {
                                "type": "number"
                            },
                            "fixed_length": {
                                "type": "number"
                            },
                            "index": {
                                "type": "number"
                            }
                        },
                        "type": "object"
                    },
                    "values": {
                        "items": {
                            "properties": {
                                "documentation": {
                                    "items": {
                                        "type": "string"
                                    },
                                    "type": "array"
                                },
                                "name": {
                                    "type": "string"
                                },
                                "object": {
                                    "properties": {
                                        "attributes": {
                                            "items": {
                                                "properties": {
                                                    "key": {
                                                        "type": "string"
                                                    },
                                                    "value": {
                                                        "type": "string"
                                                    }
                                                },
                                                "type": "object"
                                            },
                                            "type": "array"
                                        },
                                        "bytesize": {
                                            "type": "number"
                                        },
                                        "declaration_file": {
                                            "type": "string"
                                        },
                                        "documentation": {
                                            "items": {
                                                "type": "string"
                                            },
                                            "type": "array"
                                        },
                                        "fields": {
                                            "items": {
                                                "properties": {
                                                    "attributes": {
                                                        "items": {
                                                            "properties": {
                                                                "key": {
                                                                    "type": "string"
                                                                },
                                                                "value": {
                                                                    "type": "string"
                                                                }
                                                            },
                                                            "type": "object"
                                                        },
                                                        "type": "array"
                                                    },
                                                    "default_integer": {
                                                        "type": "number"
                                                    },
                                                    "default_real": {
                                                        "type": "number"
                                                    },
                                                    "deprecated": {
                                                        "type": "boolean"
                                                    },
                                                    "documentation": {
                                                        "items": {
                                                            "type": "string"
                                                        },
                                                        "type": "array"
                                                    },
                                                    "id": {
                                                        "type": "number"
                                                    },
                                                    "key": {
                                                        "type": "boolean"
                                                    },
                                                    "name": {
                                                        "type": "string"
                                                    },
                                                    "offset": {
                                                        "type": "number"
                                                    },
                                                    "optional": {
                                                        "type": "boolean"
                                                    },
                                                    "padding": {
                                                        "type": "number"
                                                    },
                                                    "required": {
                                                        "type": "boolean"
                                                    },
                                                    "type": {
                                                        "properties": {
                                                            "base_size": {
                                                                "type": "number"
                                                            },
                                                            "base_type": {
                                                                "type": "number"
                                                            },
                                                            "element": {
                                                                "type": "number"
                                                            },
                                                            "element_size": {
                                                                "type": "number"
                                                            },
                                                            "fixed_length": {
                                                                "type": "number"
                                                            },
                                                            "index": {
                                                                "type": "number"
                                                            }
                                                        },
                                                        "type": "object"
                                                    }
                                                },
                                                "type": "object"
                                            },
                                            "type": "array"
                                        },
                                        "is_struct": {
                                            "type": "boolean"
                                        },
                                        "minalign": {
                                            "type": "number"
                                        },
                                        "name": {
                                            "type": "string"
                                        }
                                    },
                                    "type": "object"
                                },
                                "union_type": {
                                    "properties": {
                                        "base_size": {
                                            "type": "number"
                                        },
                                        "base_type": {
                                            "type": "number"
                                        },
                                        "element": {
                                            "type": "number"
                                        },
                                        "element_size": {
                                            "type": "number"
                                        },
                                        "fixed_length": {
                                            "type": "number"
                                        },
                                        "index": {
                                            "type": "number"
                                        }
                                    },
                                    "type": "object"
                                },
                                "value": {
                                    "type": "number"
                                }
                            },
                            "type": "object"
                        },
                        "type": "array"
                    }
                },
                "type": "object"
            },
            "type": "array"
        },
        "fbs_files": {
            "items": {
                "properties": {
                    "filename": {
                        "type": "string"
                    },
                    "included_filenames": {
                        "items": {
                            "type": "string"
                        },
                        "type": "array"
                    }
                },
                "type": "object"
            },
            "type": "array"
        },
        "file_ext": {
            "type": "string"
        },
        "file_ident": {
            "type": "string"
        },
        "objects": {
            "items": {
                "properties": {
                    "attributes": {
                        "items": {
                            "properties": {
                                "key": {
                                    "type": "string"
                                },
                                "value": {
                                    "type": "string"
                                }
                            },
                            "type": "object"
                        },
                        "type": "array"
                    },
                    "bytesize": {
                        "type": "number"
                    },
                    "declaration_file": {
                        "type": "string"
                    },
                    "documentation": {
                        "items": {
                            "type": "string"
                        },
                        "type": "array"
                    },
                    "fields": {
                        "items": {
                            "properties": {
                                "attributes": {
                                    "items": {
                                        "properties": {
                                            "key": {
                                                "type": "string"
                                            },
                                            "value": {
                                                "type": "string"
                                            }
                                        },
                                        "type": "object"
                                    },
                                    "type": "array"
                                },
                                "default_integer": {
                                    "type": "number"
                                },
                                "default_real": {
                                    "type": "number"
                                },
                                "deprecated": {
                                    "type": "boolean"
                                },
                                "documentation": {
                                    "items": {
                                        "type": "string"
                                    },
                                    "type": "array"
                                },
                                "id": {
                                    "type": "number"
                                },
                                "key": {
                                    "type": "boolean"
                                },
                                "name": {
                                    "type": "string"
                                },
                                "offset": {
                                    "type": "number"
                                },
                                "optional": {
                                    "type": "boolean"
                                },
                                "padding": {
                                    "type": "number"
                                },
                                "required": {
                                    "type": "boolean"
                                },
                                "type": {
                                    "properties": {
                                        "base_size": {
                                            "type": "number"
                                        },
                                        "base_type": {
                                            "type": "number"
                                        },
                                        "element": {
                                            "type": "number"
                                        },
                                        "element_size": {
                                            "type": "number"
                                        },
                                        "fixed_length": {
                                            "type": "number"
                                        },
                                        "index": {
                                            "type": "number"
                                        }
                                    },
                                    "type": "object"
                                }
                            },
                            "type": "object"
                        },
                        "type": "array"
                    },
                    "is_struct": {
                        "type": "boolean"
                    },
                    "minalign": {
                        "type": "number"
                    },
                    "name": {
                        "type": "string"
                    }
                },
                "type": "object"
            },
            "type": "array"
        },
        "root_table": {
            "properties": {
                "attributes": {
                    "items": {
                        "properties": {
                            "key": {
                                "type": "string"
                            },
                            "value": {
                                "type": "string"
                            }
                        },
                        "type": "object"
                    },
                    "type": "array"
                },
                "bytesize": {
                    "type": "number"
                },
                "declaration_file": {
                    "type": "string"
                },
                "documentation": {
                    "items": {
                        "type": "string"
                    },
                    "type": "array"
                },
                "fields": {
                    "items": {
                        "properties": {
                            "attributes": {
                                "items": {
                                    "properties": {
                                        "key": {
                                            "type": "string"
                                        },
                                        "value": {
                                            "type": "string"
                                        }
                                    },
                                    "type": "object"
                                },
                                "type": "array"
                            },
                            "default_integer": {
                                "type": "number"
                            },
                            "default_real": {
                                "type": "number"
                            },
                            "deprecated": {
                                "type": "boolean"
                            },
                            "documentation": {
                                "items": {
                                    "type": "string"
                                },
                                "type": "array"
                            },
                            "id": {
                                "type": "number"
                            },
                            "key": {
                                "type": "boolean"
                            },
                            "name": {
                                "type": "string"
                            },
                            "offset": {
                                "type": "number"
                            },
                            "optional": {
                                "type": "boolean"
                            },
                            "padding": {
                                "type": "number"
                            },
                            "required": {
                                "type": "boolean"
                            },
                            "type": {
                                "properties": {
                                    "base_size": {
                                        "type": "number"
                                    },
                                    "base_type": {
                                        "type": "number"
                                    },
                                    "element": {
                                        "type": "number"
                                    },
                                    "element_size": {
                                        "type": "number"
                                    },
                                    "fixed_length": {
                                        "type": "number"
                                    },
                                    "index": {
                                        "type": "number"
                                    }
                                },
                                "type": "object"
                            }
                        },
                        "type": "object"
                    },
                    "type": "array"
                },
                "is_struct": {
                    "type": "boolean"
                },
                "minalign": {
                    "type": "number"
                },
                "name": {
                    "type": "string"
                }
            },
            "type": "object"
        },
        "services": {
            "items": {
                "properties": {
                    "attributes": {
                        "items": {
                            "properties": {
                                "key": {
                                    "type": "string"
                                },
                                "value": {
                                    "type": "string"
                                }
                            },
                            "type": "object"
                        },
                        "type": "array"
                    },
                    "calls": {
                        "items": {
                            "properties": {
                                "attributes": {
                                    "items": {
                                        "properties": {
                                            "key": {
                                                "type": "string"
                                            },
                                            "value": {
                                                "type": "string"
                                            }
                                        },
                                        "type": "object"
                                    },
                                    "type": "array"
                                },
                                "documentation": {
                                    "items": {
                                        "type": "string"
                                    },
                                    "type": "array"
                                },
                                "name": {
                                    "type": "string"
                                },
                                "request": {
                                    "properties": {
                                        "attributes": {
                                            "items": {
                                                "properties": {
                                                    "key": {
                                                        "type": "string"
                                                    },
                                                    "value": {
                                                        "type": "string"
                                                    }
                                                },
                                                "type": "object"
                                            },
                                            "type": "array"
                                        },
                                        "bytesize": {
                                            "type": "number"
                                        },
                                        "declaration_file": {
                                            "type": "string"
                                        },
                                        "documentation": {
                                            "items": {
                                                "type": "string"
                                            },
                                            "type": "array"
                                        },
                                        "fields": {
                                            "items": {
                                                "properties": {
                                                    "attributes": {
                                                        "items": {
                                                            "properties": {
                                                                "key": {
                                                                    "type": "string"
                                                                },
                                                                "value": {
                                                                    "type": "string"
                                                                }
                                                            },
                                                            "type": "object"
                                                        },
                                                        "type": "array"
                                                    },
                                                    "default_integer": {
                                                        "type": "number"
                                                    },
                                                    "default_real": {
                                                        "type": "number"
                                                    },
                                                    "deprecated": {
                                                        "type": "boolean"
                                                    },
                                                    "documentation": {
                                                        "items": {
                                                            "type": "string"
                                                        },
                                                        "type": "array"
                                                    },
                                                    "id": {
                                                        "type": "number"
                                                    },
                                                    "key": {
                                                        "type": "boolean"
                                                    },
                                                    "name": {
                                                        "type": "string"
                                                    },
                                                    "offset": {
                                                        "type": "number"
                                                    },
                                                    "optional": {
                                                        "type": "boolean"
                                                    },
                                                    "padding": {
                                                        "type": "number"
                                                    },
                                                    "required": {
                                                        "type": "boolean"
                                                    },
                                                    "type": {
                                                        "properties": {
                                                            "base_size": {
                                                                "type": "number"
                                                            },
                                                            "base_type": {
                                                                "type": "number"
                                                            },
                                                            "element": {
                                                                "type": "number"
                                                            },
                                                            "element_size": {
                                                                "type": "number"
                                                            },
                                                            "fixed_length": {
                                                                "type": "number"
                                                            },
                                                            "index": {
                                                                "type": "number"
                                                            }
                                                        },
                                                        "type": "object"
                                                    }
                                                },
                                                "type": "object"
                                            },
                                            "type": "array"
                                        },
                                        "is_struct": {
                                            "type": "boolean"
                                        },
                                        "minalign": {
                                            "type": "number"
                                        },
                                        "name": {
                                            "type": "string"
                                        }
                                    },
                                    "type": "object"
                                },
                                "response": {
                                    "properties": {
                                        "attributes": {
                                            "items": {
                                                "properties": {
                                                    "key": {
                                                        "type": "string"
                                                    },
                                                    "value": {
                                                        "type": "string"
                                                    }
                                                },
                                                "type": "object"
                                            },
                                            "type": "array"
                                        },
                                        "bytesize": {
                                            "type": "number"
                                        },
                                        "declaration_file": {
                                            "type": "string"
                                        },
                                        "documentation": {
                                            "items": {
                                                "type": "string"
                                            },
                                            "type": "array"
                                        },
                                        "fields": {
                                            "items": {
                                                "properties": {
                                                    "attributes": {
                                                        "items": {
                                                            "properties": {
                                                                "key": {
                                                                    "type": "string"
                                                                },
                                                                "value": {
                                                                    "type": "string"
                                                                }
                                                            },
                                                            "type": "object"
                                                        },
                                                        "type": "array"
                                                    },
                                                    "default_integer": {
                                                        "type": "number"
                                                    },
                                                    "default_real": {
                                                        "type": "number"
                                                    },
                                                    "deprecated": {
                                                        "type": "boolean"
                                                    },
                                                    "documentation": {
                                                        "items": {
                                                            "type": "string"
                                                        },
                                                        "type": "array"
                                                    },
                                                    "id": {
                                                        "type": "number"
                                                    },
                                                    "key": {
                                                        "type": "boolean"
                                                    },
                                                    "name": {
                                                        "type": "string"
                                                    },
                                                    "offset": {
                                                        "type": "number"
                                                    },
                                                    "optional": {
                                                        "type": "boolean"
                                                    },
                                                    "padding": {
                                                        "type": "number"
                                                    },
                                                    "required": {
                                                        "type": "boolean"
                                                    },
                                                    "type": {
                                                        "properties": {
                                                            "base_size": {
                                                                "type": "number"
                                                            },
                                                            "base_type": {
                                                                "type": "number"
                                                            },
                                                            "element": {
                                                                "type": "number"
                                                            },
                                                            "element_size": {
                                                                "type": "number"
                                                            },
                                                            "fixed_length": {
                                                                "type": "number"
                                                            },
                                                            "index": {
                                                                "type": "number"
                                                            }
                                                        },
                                                        "type": "object"
                                                    }
                                                },
                                                "type": "object"
                                            },
                                            "type": "array"
                                        },
                                        "is_struct": {
                                            "type": "boolean"
                                        },
                                        "minalign": {
                                            "type": "number"
                                        },
                                        "name": {
                                            "type": "string"
                                        }
                                    },
                                    "type": "object"
                                }
                            },
                            "type": "object"
                        },
                        "type": "array"
                    },
                    "declaration_file": {
                        "type": "string"
                    },
                    "documentation": {
                        "items": {
                            "type": "string"
                        },
                        "type": "array"
                    },
                    "name": {
                        "type": "string"
                    }
                },
                "type": "object"
            },
            "type": "array"
        }
    },
    "type": "object"
})json",
            schema_json);
}

}  // namespace aos::testing
