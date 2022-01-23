package go_greeter

import "testing"

func TestGreetings(t *testing.T) {
	cases := []struct {
		input, expected string
	}{
		{"world", "Hello, world"},
		{"  foobar", "Hello,   foobar"},
	}
	for _, c := range cases {
		greeting := Greet(c.input)
		if greeting != c.expected {
			t.Errorf("Got %q, but expected %q.", greeting, c.expected)
		}
	}
}
