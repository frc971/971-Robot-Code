require 'net/http'
begin
  require "sha1"
rescue LoadError
  require "digest/sha1"
end

require 'rubygems'
require 'nokogiri'

def retrieve_url url
  cache_file = '.url_cache/' + Digest::SHA1.hexdigest(url)
  if File.exists? cache_file
    server_response = YAML.load_file(cache_file)
  else
    uri = URI.parse(url)
    resp = Net::HTTP.start(uri.hostname, uri.port) do |http|
      path = uri.path
      if uri.query
        path += '?'
        path += uri.query
      end
      if uri.host == 'www.digikey.com'
        req = Net::HTTP::Post.new(path)
        req.set_form_data(
          'TS01ddcca6_id' => '3',
          'TS01ddcca6_cr' => 'ce1fcbf6f531dc4315cd98214a898f01:jnlm:8Ij6i51F:267420643',
          'TS01ddcca6_76' => '0',
          'TS01ddcca6_86' => '0',
          'TS01ddcca6_md' => '1',
          'TS01ddcca6_rf' => '0',
          'TS01ddcca6_ct' => '0',
          'TS01ddcca6_pd' => '0',
        )
        req['Referrer'] = 'http://www.digikey.com/product-search/en?keywords=CL21C271JBANNNC'
      else
        req = Net::HTTP::Get.new(path)
      end
      http.request(req)
    end
    if resp.code == '301' || resp.code == '302'
      server_response = retrieve_url(uri.scheme + '://' + uri.host + resp['location'])
    else
      resp.value
      server_response = resp.body
    end
    File.open(cache_file, 'w') do |f|
      YAML.dump server_response, f
    end
  end

  server_response
end
