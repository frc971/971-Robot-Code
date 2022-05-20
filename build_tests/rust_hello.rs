extern crate hello_lib;

extern "C" {
    fn sqrt(x: f64) -> f64;
}

fn main() {
    let hello = hello_lib::Greeter::new("Hello");
    println!("{},\n{}", hello.greet("world"), hello.greet("bazel"));

    let mut numbers = Vec::new();
    for i in 1..=10 {
        numbers.push(i);
    }
    println!("{:?}", numbers);

    let words = vec!["foo", "bar", "baz"];
    println!("{:?}", words);

    println!("sqrt(4) = {}", unsafe { sqrt(4.0) });
}
