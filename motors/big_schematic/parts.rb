=begin
parts.yaml is a YAML file of mappings to mappings. The top-level key is the part
number that shows up in the schematics and the second-level mappings are
overrides for various things that can't be mapped automatically.

Overrides:
  package: The PCB-style package name.
=end

require 'json'
require 'yaml'
require 'net/http'

#require './html'

def get_octopart_results(mpn)
  query = [{:mpn => mpn, :limit => 20}]
  response_include = ['specs', 'descriptions', 'short_description', 'compliance_documents', 'external_links']

  url = 'http://octopart.com/api/v3/parts/match?'
  url += '&apikey=80a2e435'
  response_include.each do |i|
    url += '&include[]=' + i
  end
  url += '&queries=' + URI.encode(JSON.generate(query))

  server_response = JSON.parse(retrieve_url(url))

  server_response['results']
end
