require 'digest'
require 'fileutils'

def javaify name
	name = name.dup
	name.gsub! /(\w)_(\w)/ do
		$1 + $2.upcase
	end
	name.gsub /^\w/ do |char|
		char.downcase
	end
end

module Contents
	class SyntaxError < Exception
	end
	class Tokenizer
		def initialize file
			@file = file
			@token = ""
			@lineno = 0
		end
		def filename
			@file.path
		end
		def pop_char
			if char = @hold_char
				@hold_char = nil
				return char
			end
			return @file.read(1)
		end
		def unpop_char char
			@hold_char = char
		end
		def clear_comment
			@hold_char = nil
			@file.gets
			@lineno += 1
		end
		def syntax_error error
			filename = File.basename(@file.path)
			line = @lineno + 1
			raise SyntaxError, error + "\n from #{line} of #{filename}", caller
		end
		def item_missing item
			syntax_error "expected \"#{item}\"! you missing something!?"
		end
		def peek_token
			@peek_token = next_token
		end
		def next_token
			if token = @peek_token
				@peek_token = nil
				return token
			end
			token = @token
			while char = pop_char
				if char == "\n"
					@lineno += 1
				end
				if char == "/"
					if pop_char == "/"
						clear_comment
					else
						syntax_error("unexpected #{char.inspect}")
					end
				elsif char =~ /[\s\r\n]/
					if token.length > 0
						@token = ""
						return token
					end
				elsif char =~ /[;\{\}]/
					if token.length > 0
						unpop_char char
						@token = ""
						return token
					end
					return(char)
				elsif token.length > 0 && char =~ /[\w:]/
					token += char
				elsif char =~ /[a-zA-Z0-9]/
					token = char
				else
					syntax_error("unexpected #{char.inspect}")
				end
			end
		rescue EOFError
		end
		def self.is_string token
			token =~ /[a-zA-Z]\w*/
		end
		def self.is_number token
			token =~ /[0-9]*/
		end
	end

	class Struct
		class StructField
			def initialize
				@members = [] # array of strings
			end
			def parse tokenizer
				while true
					token = tokenizer.next_token
					if Tokenizer.is_string(token)
						@members.push token
					elsif token == ";" || token == "\n"
						if @members.length > 0
							return @members
						else
							return nil
						end
					else
						tokenizer.syntax_error("expected member name in struct!")
					end
				end
			end
			def self.parse *args
				self.new.parse *args
			end
			def use members
				members
			end
			def self.use *args
				self.new.use *args
			end
			def to_s
				@members.join " "
			end
		end

		def parse tokenizer, parse_name = true
			if parse_name
				token = tokenizer.next_token
				if Tokenizer.is_string(token)
					@name_raw = token
				else
					tokenizer.syntax_error("expected struct name!")
				end
			else
				@name_raw = nil
				@name_data = tokenizer.filename
			end
			token = tokenizer.next_token
			tokenizer.syntax_error("expected '{', got '#{token}'") if(token != "{")
			while token != "}"
				token = tokenizer.peek_token
				if token != "}"
					field = StructField.parse(tokenizer)
					@fields.push(field) if(field)
				end
			end
			if tokenizer.next_token == "}"
				return self
			else
				tokenizer.syntax_error("wahh; call parker. #{__LINE__}")
			end
		end
		def self.parse *args
			self.new.parse *args
		end

		def use fields, name_data
			@name_raw = nil
			@name_data = name_data
			fields.each do |field|
				@fields.push(StructField.use field.split(' '))
			end
			self
		end
		def self.use *args
			self.new.use *args
		end

		def name
			@name_raw || gen_name
		end
		def gen_name
			unless @generated_name
				@generated_name = 'a' + Digest::SHA1.hexdigest(@fields.join('') + $namespace + @name_data)
			end
			@generated_name
		end

		def initialize
			@fields = [] # array of arrays of strings
			@hidden_fields = []
		end
		def upcase_name
			name.gsub(/^[a-z]|_[a-z]/) do |v|
				v[-1].chr.upcase
			end
		end
		def join_fields array
			(array.collect { |a|
				"  #{a.join(" ")};"
			}).join("\n")
		end
		def fields
			join_fields @fields
		end
		def hidden_fields
			join_fields @hidden_fields
		end
		def add_hidden_field k, v
			@hidden_fields.push [k, v]
		end
		def params
			(@fields.collect do |a|
				a.join(" ")
			end).join(', ')
		end
		def copy_params_into varname, decl = true
			(decl ? "#{name} #{varname};\n" : '') + (@fields.collect do |a|
				"#{varname}.#{a[-1]} = #{a[-1]};"
			end).join("\n")
		end
		def params_from name
			(@fields.collect do |a|
				name + '.' + a[-1]
			end).join(', ')
		end
		def builder_name aos_namespace = true, this_namespace = true
			(aos_namespace ? "aos::" : '') + "QueueBuilder<#{this_namespace ? $namespace + '::' : ''}#{name}>"
		end
		def java_builder
			name + 'Builder'
		end
		def builder_defs name
			(@fields.collect do |field|
				"  inline #{name} &#{field[-1]}" +
				"(#{field[0...-1].join(" ")} in) " +
				"{ holder_.View().#{field[-1]} = in; return *this; }"
			end).join "\n"
		end
		def swig_builder_defs name
			(@fields.collect do |field|
				"  %rename(#{javaify field[-1]}) #{field[-1]};\n" +
				"  #{name} &#{field[-1]}" +
				"(#{field[0...-1].join(" ")} #{field[-1]});"
			end).join "\n"
		end
		def zero name
			(@fields.collect do |field|
				"    new (&#{name}.#{field[-1]}) #{field[0...-1].join ' '}();"
			end).join("\n")
		end
		def size
			(@fields.collect do |field|
				"sizeof(#{$namespace}::#{name}::#{field[-1]})"
			end.push('0')).join(' + ')
		end
		def get_format(field)
			case(field[0...-1])
			when ['int']
				r = '%d'
			when ['float'], ['double']
				r = '%f'
			when ['bool']
				r = '%s'
			when ['uint8_t']
				r = '%hhu'
			when ['uint16_t']
				r = '%d'
			when ['struct', 'timespec']
				r = '%jdsec,%ldnsec'
			else
				return 'generator_error'
			end
			return field[-1] + ': ' + r
		end
		def to_printf(name, field)
			case(field[0...-1])
			when ['bool']
				return name + '.' + field[-1] + ' ? "true" : "false"'
			when ['uint16_t']
				return "static_cast<int>(#{name}.#{field[-1]})"
			when ['struct', 'timespec']
				return "#{name}.#{field[-1]}.tv_sec, #{name}.#{field[-1]}.tv_nsec"
			else
				return name + '.' + field[-1]
			end
		end
		def netop name, buffer
			offset = '0'
			(@fields.collect do |field|
				# block |var_pointer, output_pointer|
				val = yield "&#{name}.#{field[-1]}", "&#{buffer}[#{offset}]"
				offset += " + sizeof(#{name}.#{field[-1]})"
				'    ' + val
			end).join("\n") + "\n    " +
				"static_assert(#{offset} == #{size}, \"code generator issues\");"
		end
		def hton name, output
			netop(name, output) do |var, output|
				"to_network(#{var}, #{output});"
			end
		end
		def ntoh input, name
			netop(name, input) do |var, input|
				"to_host(#{input}, #{var});"
			end
		end
		def swig_writer
			<<END
struct #{name} {
#{(@fields.collect { |a|
	"  %rename(#{javaify a[-1]}) #{a[-1]};"
}).join("\n")}
#{self.fields}
  %extend {
    const char *toString() {
      return aos::TypeOperator<#{$namespace}::#{name}>::Print(*$self);
    }
  }
 private:
  #{name}();
};
} // namespace #{$namespace}
namespace aos {
%typemap(jstype) #{builder_name false}& "#{java_builder}"
%typemap(javaout) #{builder_name false}& {
    $jnicall;
    return this;
  }
template <> class #{builder_name false} {
 private:
  #{builder_name false}();
 public:
  inline bool Send();
  %rename(#{javaify 'Send'}) Send;
#{swig_builder_defs builder_name(false)}
};
%template(#{java_builder}) #{builder_name false};
%typemap(javaout) #{builder_name false}& {
    return new #{java_builder}($jnicall, false);
  }
} // namespace aos
namespace #{$namespace} {
END
		end
		def writer
			<<END
struct #{name} {
#{self.fields}
#{self.hidden_fields}
};
} // namespace #{$namespace}
namespace aos {
template <> class TypeOperator<#{$namespace}::#{name}> {
 public:
  static void Zero(#{$namespace}::#{name} &inst) {
    (void)inst;
#{zero 'inst'}
  }
  static void NToH(const char *input, #{$namespace}::#{name} &inst) {
    (void)input;
    (void)inst;
#{ntoh 'input', 'inst'}
  }
  static void HToN(const #{$namespace}::#{name} &inst, char *output) {
    (void)inst;
    (void)output;
#{hton 'inst', 'output'}
  }
  static inline size_t Size() { return #{size}; }
  static const char *Print(const #{$namespace}::#{name} &inst) {
#{@fields.empty? ? <<EMPTYEND : <<NOTEMPTYEND}
    (void)inst;
    return "";
EMPTYEND
    static char buf[1024];
    if (snprintf(buf, sizeof(buf), "#{@fields.collect do |field|
	    get_format(field)
    end.join(', ')}", #{@fields.collect do |field|
	    to_printf('inst', field)
    end.join(', ')}) >= static_cast<ssize_t>(sizeof(buf))) {
      LOG(WARNING, "#{name}'s buffer was too small\\n");
      buf[sizeof(buf) - 1] = '\\0';
    }
    return buf;
NOTEMPTYEND
  }
};
template <> class #{builder_name false} {
 private:
  aos::QueueHolder<#{$namespace}::#{name}> &holder_;
 public:
  #{builder_name false}(aos::QueueHolder<#{$namespace}::#{name}> &holder) : holder_(holder) {}
  inline bool Send() { return holder_.Send(); }
  inline const char *Print() const { return holder_.Print(); }
#{builder_defs builder_name(false)}
};
} // namespace aos
namespace #{$namespace} {
END
		end
		def to_s
			return <<END
#{name}: #{(@fields.collect {|n| n.join(" ") }).join("\n\t")}
END
		end
	end

	class SimpleField
		def initialize check_function = :is_string
			@check_function = check_function
			@name = nil
		end
		def parse tokenizer
			token = tokenizer.next_token
			if Tokenizer.__send__ @check_function, token
				@name = token
			else
				tokenizer.syntax_error('expected value!')
			end
			if tokenizer.next_token == ';'
				@name
			else
				tokenizer.syntax_error('expected ";"!')
			end
		end
		def self.parse tokenizer
			self.new.parse tokenizer
		end
	end
	class NameField < SimpleField
	end

	class OutputFile
		def initialize namespace, filename, topdir, outpath
			@namespace = namespace
			$namespace = namespace
			@base = filename.gsub(/\.\w*$/, "").gsub(/^.*\//, '')
      @topdir = topdir
			@filebase = outpath + @base
			@filename = filename
      FileUtils.mkdir_p(outpath)

			fillin_initials if respond_to? :fillin_initials
			parse filename
			fillin_defaults if respond_to? :fillin_defaults
			self
		rescue SyntaxError => e
			puts e
			exit 1
		end
		def filename type
			case type
			when 'h'
				@filebase + '.q.h'
			when 'cc'
				@filebase + '.q.cc'
			when 'main'
				@filebase + '_main.cc'
			when 'swig'
				@filebase + '.swg'
      when 'java_dir'
        @filebase + '_java/'
      when 'java_wrap'
        @filebase + '_java_wrap.cc'
			else
				throw SyntaxError, "unknown filetype '#{type}'"
			end
		end
		def parse filename
			file = File.open filename
			tokenizer = Tokenizer.new file
			while token = tokenizer.next_token
				if !token || token.gsub('\s', '').empty?
				elsif token == ';'
				else
					error = catch :syntax_error do
						case token
						when 'namespace'
							$namespace = NameField.parse tokenizer
						else
							parse_token token, tokenizer
						end
						nil
					end
					if error
						tokenizer.syntax_error error.to_s
						raise error
					end
				end
			end

			check_format tokenizer
		end
    def call_swig
      output_dir = filename('java_dir') + $namespace
      FileUtils.mkdir_p(output_dir)
      if (!system('swig', '-c++', '-Wall', '-Wextra', '-java',
                  '-package', $namespace, "-I#{@topdir}",
                  '-o', filename('java_wrap'),
                  '-outdir', output_dir, filename('swig')))
        exit $?.to_i
      end
    end

		def queue_holder_accessors suffix, var, type, force_timing = nil
<<END
  inline bool Get#{suffix}(#{force_timing ? '' : 'bool check_time'}) aos_check_rv { return #{var}.Get(#{force_timing || 'check_time'}); }
  inline #{type} &View#{suffix}() { return #{var}.View(); }
  inline void Clear#{suffix}() { #{var}.Clear(); }
  inline bool Send#{suffix}() { return #{var}.Send(); }
  inline const char *Print#{suffix}() { return #{var}.Print(); }
  inline aos::QueueBuilder<#{type}> &#{suffix || 'Builder'}() { return #{var}.Builder(); }
END
		end
		def queue_holder_accessors_swig suffix, var, type, force_timing = nil
<<END
  %rename(#{javaify "Get#{suffix}"}) Get#{suffix};
  bool Get#{suffix}(#{force_timing ? '' : 'bool check_time'});
  %rename(#{javaify "View#{suffix}"}) View#{suffix};
  #{type} &View#{suffix}();
  %rename(#{javaify "Clear#{suffix}"}) Clear#{suffix};
  void Clear#{suffix}();
  %rename(#{javaify "Send#{suffix}"}) Send#{suffix};
  bool Send#{suffix}();
  %rename(#{javaify suffix || 'Builder'}) #{suffix || 'Builder'};
  aos::QueueBuilder<#{type}> &#{suffix || 'Builder'}();
END
		end
	end
end

def write_file_out
	if ARGV.length < 3
		puts 'Error: at least 3 arguments required!!!'
		exit 1
	end
	file = Contents::OutputFile.new ARGV.shift,
		File.expand_path(ARGV.shift),
    ARGV.shift,
		File.expand_path(ARGV.shift) + '/'
	while !ARGV.empty?
		case type = ARGV.shift
		when 'cpp'
			file.write_cpp
		when 'header'
			file.write_header
		when 'main'
			file.write_main
		when 'swig'
			file.write_swig
      file.call_swig
		else
			puts "Error: unknown output type '#{type}'"
			exit 1
		end
	end
end

