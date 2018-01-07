#!/usr/bin/env ruby

# This makes DNS lookups actually work reliably and give useful errors.
require 'resolv-replace.rb'

require 'yaml'
require 'optparse'

require './parts'
require './gschem_file'

=begin
Property names in the schematic file:
  value is the main value of the component.
  secondary_value is another value of the component.
    The value in the schematic is only a minimum.
      power rating for resistors.
      voltage for capacitors.
  pn_optional set to 1 means that the pn property is non-critical.
  tolerance is a maximum tolerance for value.
=end

options = {}
OptionParser.new do |opts|
  opts.banner = "Usage: ordering.rb file.sch [options]"

  opts.on('--myro-bom', String, :REQUIRED, 'Generate myropcb BOM') do |v|
    options[:bom_file] = v
    options[:bom_style] = :myro
  end
  opts.on('--mouser-bom', String, :REQUIRED, 'Generate Mouser BOM (ONLY MOUSER PARTS)') do |v|
    options[:bom_file] = v
    options[:bom_style] = :mouser
  end
  opts.on('--bom', String, :REQUIRED, 'Generate a nice BOM') do |v|
    options[:bom_file] = v
    options[:bom_style] = :nice
  end
end.parse!

options[:order_quantity] = 2
options[:include_dev] = true

if options[:bom_file]
  $bom_file = File.open(options[:bom_file], 'w')
  case options[:bom_style]
  when :myro
    $bom_file.puts 'Qty,Part Reference,Part Number,MFGR,Distributor NO,DESCRIPTION,FOOTPRINT,DNI'
  when :mouser
    $bom_file.puts 'Mfg Part Number,Quantity,Description'
  else
    $bom_file.puts 'Quantity,Refdes,Part Number,Footprint,Value,Unit Price'
  end
end

if ARGV.size == 0
  raise 'Need schematics to look at!'
end
puts ARGV
filenames = get_schematic_filenames ARGV

#$parts_yaml = YAML.load_file('parts.yaml')
$parts_yaml = {}

module Unicode
  NBSP = "\u00A0"
  PlusMinus = "\u00B1"
  Micro = "\u00B5"
end

def chomp_zeros s
  while s[-1] == '0'
    s.chomp! '0'
  end
end

class Part
  SecondaryValueRegex = /^([0-9.]+) (.*)$/

  attr_reader :refdeses
  attr_reader :pn, :value, :footprint, :device, :secondary_value, :tolerance
  attr_reader :pn_optional, :dev_only

  attr_reader :low_quantity

  def quantity
    @refdeses.size
  end

  def initialize(pn, value, footprint, device, secondary_value, refdes, tolerance, pn_optional, dev_only)
    if value == 'DNP'
      if pn
        puts "Error: part #{pn} for DNP from #{refdes}"
      end
      @value = 'DNP'
      @pn, @footprint, @device = nil, nil, nil
      @secondary_value, @tolerance = nil
      @pn_optional, @dev_only = nil
    else
      @pn, @value, @footprint, @device = pn, value, footprint, device
      @secondary_value, @tolerance = secondary_value, tolerance
      @pn_optional, @dev_only = pn_optional, dev_only

      puts "#{refdes} has no footprint" unless footprint
    end

    @refdeses = [refdes]
  end
  def found_another(pn, value, footprint, device, secondary_value, refdes, tolerance, pn_optional, dev_only)
    error = false
    if value == 'DNP'
      if @value != 'DNP'
        puts "Error: DNP vs not"
        error = true
      end
    else
      if pn != @pn
        puts "Error: pn #@pn vs #{pn}."
        error = true
      end
      if value != @value
        puts "Error: value #@value vs #{value}."
        error = true
      end
      if footprint != @footprint
        puts "Error: footprint #@footprint vs #{footprint}."
        error = true
      end
      if device != @device
        puts "Error: device #@device vs #{device}."
        error = true
      end

      if pn_optional != @pn_optional
        puts "Error: pn_optional #@pn_optional vs #{pn_optional}."
        error = true
      end
      if dev_only != @dev_only
        puts "Error: dev_only #@dev_only vs #{dev_only}."
        error = true
      end

      if tolerance && @tolerance
        if tolerance != @tolerance
          puts "Error: tolerance #@tolerance vs #{tolerance}."
          error = true
        end
      elsif tolerance
        @tolerance = tolerance
      end

      new_secondary_match = SecondaryValueRegex.match secondary_value
      my_secondary_match = SecondaryValueRegex.match @secondary_value
      if new_secondary_match && my_secondary_match &&
        new_secondary_match[2] == my_secondary_match[2]
        new_value = new_secondary_match[1].to_f
        my_value = my_secondary_match[1].to_f
        @secondary_value = [new_value, my_value].max.to_s + ' ' + my_secondary_match[2]
      else
        if secondary_value != @secondary_value
          puts "Error: secondary_value #@secondary_value vs #{secondary_value}."
          error = true
        end
      end
    end

    if error
      puts "\tAdding #{refdes} to #{@refdeses.inspect}"
    end

    @refdeses.push refdes unless @refdeses.include? refdes
  end

  def override_pn pn
    @pn = pn
  end
  def override_footprint footprint
    @footprint = footprint
  end
end

$parts = {}

def add_part(attrs, refdes)
  args = [attrs[:pn], attrs[:value], attrs[:footprint], attrs[:device],
          attrs[:secondary_value], refdes, attrs[:tolerance],
          attrs[:pn_optional], attrs[:dev_only]]
  if attrs[:value] == 'DNP'
    key = 'DNP'
  else
    key = attrs[:pn]
    unless key
      key = attrs[:value] || ''
      key += attrs[:footprint] || ''
    end
  end
  raise "No pn or value for #{args} (#{refdes})" unless key && !key.empty?
  if $parts.include?(key)
    $parts[key].found_another(*args)
  else
    $parts[key] = Part.new(*args)
  end
end

def set_or_same(old, new)
  if new
    if old
      if old != new
        raise "#{old.inspect} != #{new.inspect}"
      end
    end
    new
  else
    old
  end
end

MouserKeyRegex = /^\n(.*):\n$/
MouserValueRegex = /^\n(.*)\n$/

def mouser_table_to_map table
  raise "Bad table from Mouser" if table.empty?
  r = {}
  rows = table.css('tr > td > table > tr')
  rows.each do |row|
    raise "Unexpected HTML" unless row.elements.length == 3

    name = row.elements[0].text
    match = MouserKeyRegex.match(name)
    raise "Unexpected table key #{name.inspect}" unless match
    key = match[1]

    value = row.elements[1].text
    match = MouserValueRegex.match(value)
    raise "Unexpected table value #{value.inspect}" unless match
    value = match[1]
    r[key] = value
  end
  r
end

def digikey_table_to_map table
  raise "Unexpected HTML" unless table.name == 'table'
  r = {}
  table.elements.each do |row|
    raise "Unexpected HTML" unless row.name == 'tr'
    raise "Unexpected HTML" unless row.elements.length == 2

    name, value = row.elements
    raise "Unexpected HTML" unless name.name == 'th'
    raise "Unexpected HTML" unless value.name == 'td'
    r[name.text.gsub(Unicode::NBSP, '').strip] = value.text.gsub(Unicode::NBSP, '').strip
  end
  r
end

SimpleAttributes = [:pn, :value, :footprint, :device, :tolerance, :pn_optional, :dev_only]

filenames.each do |filename|
  puts filename
  file = GschemSchematic.new(filename)
  file.components.each do |component|
    next if component.is_power
    next if component[:graphical]

    simple = {}
    SimpleAttributes.each do |attr|
      simple[attr] = component[attr]
    end
    case component[:device]
    when 'CAPACITOR'
      simple[:secondary_value] = component[:voltage]
    when 'POLARIZED_CAPACITOR'
      simple[:secondary_value] = component[:voltage]
    when 'RESISTOR'
      simple[:secondary_value] = component[:power]
    end

    symbol_filename = "schematic/symbols/#{component.filename}"
    if File.exists? symbol_filename
      symbol = GschemSymbol.new(symbol_filename)
      if symbol[:device]
        if component[:pn]
          if !component[:footprint]
            puts "#{component.refdes} has a pn and its symbol has a device but it has no footprint."
          end
        else
          simple[:pn] = symbol[:device]
        end
      end

      SimpleAttributes.each do |attr|
        if symbol[attr]
          if component[attr]
            if component[attr] != symbol[attr]
              puts "#{component.refdes} overriding #{attr} from symbol."
              puts "\t#{symbol[attr]} => #{component[attr]}"
            end
          else
            simple[attr] = symbol[attr]
          end
        end
      end
    end

    add_part(simple, component.refdes)
  end
end

# An array of arrays where the first one is the English case code and the second
# is the metric one.
MetricCases = [['0402', '1005'], ['0603', '1608'], ['0805', '2012'],
               ['1206', '3216'], ['1210', '3225'], ['1812', '4532']]
SimpleCaseRegex = /^\d\d\d\d$/

def retrieve_mouser_package attributes
  raw = attributes['Package / Case']
  r = nil
  if raw
    if SimpleCaseRegex =~ raw
      r = set_or_same r, raw
    else
      t = nil
      MetricCases.each do |pair|
        if /^#{pair[0]} \(#{pair[1]} [mM]etric\)$/ =~ raw
          t = set_or_same r, pair[0]
        end
      end
      r = set_or_same r, t || raw
    end
  end
  r = set_or_same r, attributes['Case Code - in']

  unless r
    return nil
  end

  $parts_yaml['footprint_mappings'][r] || r
end

class PartInfo
  Price = Struct.new(:quantity, :price, :link, :dpn)

  attr_reader :part

  attr_accessor :case_package
  attr_accessor :component_value
  attr_accessor :component_secondary_value
  attr_accessor :component_value_tolerance

  attr_accessor :order_quantity

  # value_field is the field name for Octopart.
  # value_unit is the units Octopart has.
  # value_unit_suffix is the suffix in the .sch file.
  attr_accessor :value_field, :value_unit, :value_unit_suffix
  # mouser_value_unit_suffix is the suffix Mouser has on the value.
  attr_accessor :mouser_value_field, :mouser_value_unit_suffix
  attr_accessor :digikey_value_field, :digikey_value_unit_suffix
  attr_accessor :secondary_value_field, :secondary_value_unit
  attr_accessor :digikey_secondary_value_field, :mouser_secondary_value_field
  attr_accessor :mouser_secondary_value_field

  attr_accessor :manufacturer

  def initialize part
    @part = part

    @prices = []
    @descriptions = []
  end

  def add_description d
    @descriptions.push d
  end
  def description
    raise "No description for #{part}" if @descriptions.empty?
    @descriptions[0]
  end

  def set_or_same field, value
    name = '@' + field.to_s
    old = instance_variable_get name
    if value
      if old
        if value.class == Float && old.class == Float
          eq = float_eq old, value
        else
          eq = old == value
        end
        if !eq
          raise "#{old.inspect} (old) != #{value.inspect} (new) for #{part.inspect}"
        end
      else
        instance_variable_set name, value
        return value
      end
    end
    return old
  end

  def add_price price
    @prices.push price
  end
  def price_for quantity
    return nil, nil if @prices.empty?
    lowest_info = nil
    lowest_price = Float::INFINITY
    @prices.each do |price|
      c = price.price * (quantity.to_f / price.quantity.to_f).ceil * price.quantity
      if c < lowest_price
        lowest_info = price
        lowest_price = c
      end
    end
    return lowest_price, lowest_info unless lowest_info
    if lowest_info.quantity.to_f / quantity.to_f > 5 && (lowest_info.quantity - quantity) > 50
      puts "Warning: no good price for #{quantity} of #{part.inspect}"
      puts "\thave #{@prices.inspect}"
    end
    return lowest_price, lowest_info
  end
end

def strip_from_end suffix, raw
  if suffix.class == Regexp
    match = suffix.match raw
    raise "#{raw.inspect} does not match #{suffix}" unless match
    match[1]
  else
    raise "#{raw.inspect} does not end with #{suffix}" unless raw.end_with? suffix
    raw[0..-(suffix.length + 1)]
  end
end

def make_digikey_request part, info
  pn = $parts_yaml['digikey_numbers'][part.pn] || part.pn
  page = "http://www.digikey.com/product-search/en?keywords=#{URI.escape pn}"
  page = $parts_yaml[part.pn]['digikey_url'] || page if $parts_yaml[part.pn]
  results = Nokogiri::HTML(retrieve_url(page))
  search_results_html = results.css '#productTable > tbody'
  values_table_html = results.css '.attributes-table-main'
  prices_table = results.css '#pricing'
  product_details = results.css '.product-details'
  if !results.css(':contains("No records match your search criteria")').empty?
    # No results from Digikey. Skip it.
  elsif !values_table_html.empty?
    raise "Unexpected HTML" unless values_table_html.length == 1
    raise "Unexpected HTML" unless prices_table.length == 1
    raise "Unexpected HTML" unless product_details.length == 1
    parse_digikey_results values_table_html[0].elements[0], prices_table[0],
                          product_details[0], page, info
  elsif !search_results_html.empty?
    raise "Unexpected HTML" unless search_results_html.length == 1
    found = false
    search_results_html.children.each do |result|
      if result.elements.length == 0
        next
      end
      raise "Unexpected HTML" unless result.elements.length >= 5
      links = result.elements[4].css 'a'
      raise "Unexpected HTML" unless links.length == 1
      mpn = links[0].text.strip
      if mpn == part.pn
        found = true

        actual_page = 'http://www.digikey.com' + links[0]['href']
        new_results = Nokogiri::HTML(retrieve_url(actual_page))
        new_values_table = new_results.css '.attributes-table-main'
        new_prices_table = new_results.css '#pricing'
        new_product_details = new_results.css '.product-details'
        new_feedback = results.css '#product-details-feedback'
        unless new_feedback.empty?
          if new_feedback.text == 'Obsolete item; call Digi-Key for more information.'
            next
          else
            puts "Warning: Digikey gave feedback #{new_feedback}"
          end
        end
        raise "Unexpected HTML" unless new_values_table.length == 1
        raise "Unexpected HTML" unless new_prices_table.length == 1
        raise "Unexpected HTML" unless new_product_details.length == 1
        parse_digikey_results new_values_table[0].elements[0],
                              new_prices_table[0], new_product_details[0],
                              actual_page, info
      end
    end

    if !found
      puts "Warning: Got multiple Digikey search results for #{part.inspect}, none of them right."
    end
  else
    raise "Don't know what Digikey page #{page} => #{results} for #{part.inspect} contains!"
  end
end

def make_mouser_request part, info
  pn = $parts_yaml['mouser_numbers'][part.pn] || part.pn
  url = "http://www.mouser.com/Search/Refine.aspx?Keyword=#{URI.escape pn}"
  mouser_results = Nokogiri::HTML(retrieve_url(url))
  search_results_html = mouser_results.css '#ctl00_ContentMain_SearchResultsGrid_grid'
  values_table_html = mouser_results.css '.specs'
  prices_table = mouser_results.css '.PriceBreaks'
  details_table = mouser_results.css '#product-desc'
  if !mouser_results.css(':contains("did not return any results.")').empty?
    # No results from Mouser. Skip it.
  elsif !values_table_html.empty?
    parse_mouser_results values_table_html, prices_table, details_table, url, info
  elsif !search_results_html.empty?
    found = false
    search_results_html.children[2..-1].each do |result|
      if result.elements.length == 0
        next
      end
      raise "Unexpected HTML" unless result.elements.length >= 4
      links = result.elements[3].css 'a'
      raise "Unexpected HTML" unless links.length >= 1
      mpn = links.first.text.strip
      if mpn == part.pn
        if result.elements[2].text.strip == 'Not Assigned'
          # This means that Mouser doesn't actually carry it and probably
          # doesn't have much data on it, none of which I want to trust.
          next
        end
        found = true

        actual_page = 'http://www.mouser.com/Search/' + links[0]['href']
        mouser_results = Nokogiri::HTML(retrieve_url(actual_page))
        values_table_html = mouser_results.css '.specs'
        prices_table = mouser_results.css '.PriceBreaks'
        details_table = mouser_results.css '#product-desc'
        parse_mouser_results values_table_html, prices_table, details_table, actual_page, info
      end
    end
    if !found
      puts "Warning: Got multiple Mouser search results for #{part.inspect}, none of them right."
    end
  else
    raise "Don't know what Mouser page #{url} => #{mouser_results} contains!"
  end
end

WeirdFractionRegex = /^([0-9.]+ [a-zA-Z]+) \([0-9]+\/[0-9]+ [a-zA-Z]+\)$/

def parse_digikey_results values_table_html, prices_table, product_details, url, info
  attributes = digikey_table_to_map values_table_html

  digikey_package = retrieve_mouser_package attributes
  if digikey_package != 'Non Standard' && digikey_package != 'Nonstandard'
    info.set_or_same :case_package, digikey_package
  end

  if info.digikey_value_field && attributes[info.digikey_value_field]
    v = strip_from_end info.digikey_value_unit_suffix, attributes[info.digikey_value_field]
    info.set_or_same :component_value, to_f_with_suffix(v)
  end
  if info.digikey_secondary_value_field && attributes[info.digikey_secondary_value_field]
    v = strip_from_end info.secondary_value_unit, attributes[info.digikey_secondary_value_field]
    info.set_or_same :component_secondary_value, to_f_with_suffix(v)
  end

  if attributes['Tolerance']
    if attributes['Tolerance'] == '-20%, +80%'
      info.set_or_same :component_value_tolerance, 80
    else
      v = strip_from_end '%', attributes['Tolerance'].gsub(Unicode::PlusMinus, '')
      info.set_or_same :component_value_tolerance, v.to_f
    end
  end

  distributor_row = product_details.elements[2]
  raise "Unexpected HTML" unless distributor_row.elements[0].text == 'Digi-Key Part Number'
  description_row = product_details.elements[6]
  raise "Unexpected HTML" unless description_row.elements[0].text == 'Description'

  info.add_description description_row.elements[1].text.chomp

  prices_table.elements[1..-1].each do |price|
    quantity = price.elements[0].text.gsub(',', '').to_i
    raise unless quantity.to_s == price.elements[0].text.gsub(',', '')
    price_number = price.elements[1].text.gsub(',', '').to_f
    raise unless chomp_zeros(price_number.to_s) == chomp_zeros(price.elements[1].text)
    info.add_price PartInfo::Price.new(quantity, price_number, url,
                                       distributor_row.elements[1].text)
  end

  manufacturer_row = product_details.elements[4]
  raise "Unexpected HTML" unless manufacturer_row.elements[0].text == 'Manufacturer'
  manufacturer = manufacturer_row.elements[1].text
  manufacturer = $parts_yaml['manufacturer_names'][manufacturer] || manufacturer
  info.set_or_same :manufacturer, manufacturer
end

def parse_mouser_results values_table_html, prices_table, details_table, url, info
  attributes = mouser_table_to_map values_table_html

  mouser_package = retrieve_mouser_package attributes
  if info.case_package == 'SRR1210' && mouser_package == '1210'
  elsif info.case_package == '0612' && mouser_package == '1206'
  else
    info.set_or_same :case_package, mouser_package
  end

  if info.mouser_value_field && attributes[info.mouser_value_field]
    v = strip_from_end info.mouser_value_unit_suffix, attributes[info.mouser_value_field]
    info.set_or_same :component_value, to_f_with_suffix(v)
  end
  if info.mouser_secondary_value_field && attributes[info.mouser_secondary_value_field]
    v = attributes[info.mouser_secondary_value_field]
    match = WeirdFractionRegex.match v
    if match
      v = match[1]
    end
    v = strip_from_end info.secondary_value_unit, v
    info.set_or_same :component_secondary_value, to_f_with_suffix(v)
  end

  if info.part.device != 'CRYSTAL' && attributes['Tolerance']
    v = strip_from_end '%', attributes['Tolerance'].gsub('+/-', '')
    info.set_or_same :component_value_tolerance, v.to_f
  end

  raise "Unexpected HTML" unless details_table.length == 1
  details_table_contents = {}
  if details_table[0].elements.size > 1
    details_table.first.elements.each do |row|
      details_table_contents[row.elements[0].text.strip] =
        row.elements[1].text.strip
    end
  else
    details_table.first.elements.first.elements.each do |row|
      details_table_contents[row.elements.first.text.strip] =
        row.elements[1].text.strip
    end
  end
  details_table_contents.each do |_, v|
    if v == 'Obsolete'
      return
    end
  end

  manufacturers = details_table_contents['Manufacturer:'].lines.collect do |line|
    line.strip
  end.delete_if do |line|
    line.empty?
  end.uniq
  unless manufacturers.length == 1
    raise "Not sure what manufacturers #{manufactures.inspect} from Mouser mean"
  end
  manufacturer = manufacturers[0]
  manufacturer = $parts_yaml['manufacturer_names'][manufacturer] || manufacturer
  info.set_or_same :manufacturer, manufacturer

  distributor = details_table_contents['Mouser Part #:']
  if distributor == 'Not Assigned'
    return
  end
  raise "Unexpected HTML" unless prices_table.length == 1

  info.add_description details_table_contents['Description:'].chomp

  prices_table[0].elements.each do |row|
    raise "Unexpected HTML" unless row.name == 'div' || row.name == 'tr'
    if row.elements.length == 4
      quantity_text = row.elements[1].elements[0].text
      price_text = row.elements[2].text.strip
      if price_text == 'Quote>'
        next
      elsif price_text == ''
        # There is (sometimes?) an empty row at the bottom.
        raise "Huh?" unless quantity_text == ''
        next
      elsif !price_text.start_with? '$'
        puts row
        raise "Don't know how to interpret Mouser price #{price_text.inspect}."
      end
      quantity = quantity_text.gsub(',', '').to_i
      raise unless quantity.to_s == quantity_text.gsub(',', '')
      price = price_text[1..-1].gsub(',', '').to_f
      raise unless chomp_zeros(price.to_s) == chomp_zeros(price_text[1..-1])
      info.add_price PartInfo::Price.new(quantity, price, url, distributor)
    end
  end
end

def parse_value f, unit
  raise "Don't know what min/max values mean." if f['min_value'] || f['max_value']
  v = f['value'][0]
  if unit == '%'
    raise "Want %, not units" if f['metadata']['unit']
    if v == '+80/-20%'
      v = '80'
    else
      v = strip_from_end '%', v.gsub(Unicode::PlusMinus, '')
    end
  else
    if f['metadata']['unit']['name'] != unit
      raise "Wrong units on #{f.inspect} (expected #{unit})."
    end
  end
  raise "Don't know what multiple values #{f['value'].uniq} mean." if f['value'].uniq.size != 1
  r = v.to_f
  raise "Error parsing #{v}" if r == 0.0
  r
end

SiSuffixes = {'M' => 1e6, 'k' => 1e3, 'm' => 1e-3, 'u' => 1e-6, 'n' => 1e-9,
              'p' => 1e-12, 'f' => 1e-15,
              Unicode::Micro => 1e-6}

# Matches Digikey-style resistor power ratings like "0.063W, 1/16".
FractionalWattsRegex = /^([0-9.]+)W, ([0-9]+)\/([0-9]+)$/

def to_f_with_suffix raw
  raw.rstrip!
  suffix_power = 1
  SiSuffixes.each do |suffix, power|
    if raw.end_with? suffix
      raw = raw[0..-2]
      suffix_power *= power
    end
  end
  raw.rstrip!

  fractional_watts = FractionalWattsRegex.match(raw)
  if fractional_watts
    raise "Fraction and power for #{raw.inspect}?" unless suffix_power == 1
    decimal = fractional_watts[1].to_f
    fraction = fractional_watts[2].to_f / fractional_watts[3].to_f
    if (decimal - fraction).abs > 10 ** -(fractional_watts[1].length - 2)
      raise "Mismatched fraction #{fraction} and decimal #{decimal}"
    end
    return decimal
  end

  raise "Can't parse number #{raw.inspect}" if /^[0-9.]+$/ !~ raw

  raw.to_f * suffix_power
end

def float_eq a, b
  ((a - b).abs / [a.abs, b.abs].max) < 1e-15
end

def check_value_attribute raw, suffix, expected
  raw = strip_from_end suffix, raw
  found_value = to_f_with_suffix raw
  float_eq expected, found_value
end

total_price = 0
total_per_item_price = 0

$parts.each do |_, part|
  info = PartInfo.new part

  unit_price = nil
  order_quantity = nil
  if $bom_file
    #footprint = $parts_yaml['bom_footprints'][part.footprint] || part.footprint
    footprint = part.footprint
    case options[:bom_style]
    when :myro
      line = [part.quantity,
              part.refdeses.join(' '),
              part.pn,
              info.manufacturer,
              lowest_price_info ? lowest_price_info.dpn : 'NA',
              info.description.gsub('"', ''),
              footprint,
              dnp ? 'DNI' : ''].collect do |o|
        o.to_s.inspect
      end.join(',')
    when :mouser
      if lowest_price_info.link.include?('mouser.com')
        #line = "#{part.pn.inspect},#{order_quantity},#{info.description.gsub('"', '').inspect}"
      elsif lowest_price_info.link.include?('digikey.com')
        line = "#{lowest_price_info.dpn},#{order_quantity}"
      else
        raise
      end
    else
      line = [part.quantity,
              part.refdeses.join(' '),
              part.pn,
              footprint,
              part.value,
              unit_price].collect do |o|
        o.to_s.inspect
      end.join(',')
    end
    $bom_file.puts line if line
  end
end
puts total_price
puts total_per_item_price

$bom_file.close if $bom_file
