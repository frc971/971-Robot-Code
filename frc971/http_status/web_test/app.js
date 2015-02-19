/*
From Seasocks version 1.1.2, under /src/ws_test_web

Copyright (c) 2013, Matt Godbolt
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this 
list of conditions and the following disclaimer in the documentation and/or 
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

var ws;

$(function() {
    ws = new WebSocket('ws://' + document.location.host + '/ws');
    ws.onopen = function() {
        console.log('onopen');
    };
    ws.onclose = function() {
        $('#message').text('Lost connection.');
        console.log('onclose');
    };
    ws.onmessage = function(message) {
        console.log("got '" + message.data + "'");
        eval(message.data);
    };
    ws.onerror = function(error) {
        console.log('onerror ' + error);
        console.log(error);
    };
    $('#count').click(function() {
      ws.send($('#count').val());
    });
    $('#close').click(function() {
      ws.send('close');
    });
    $('#die').click(function() {
      ws.send('die');
    });
});

set = function(value) {
  $('#count').val(value)
}
