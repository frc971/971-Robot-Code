extern "C" {
    fn c_return5() -> u8;
    fn c_take5(x: *mut u8);
}

#[no_mangle]
pub unsafe extern "C" fn rust_return5() -> i32 {
    let layout = std::alloc::Layout::from_size_align(1, 1).unwrap();
    let a = std::alloc::alloc(layout);
    *a = c_return5();
    // Do something so the compiler can't optimize out the alloc+free pair, which would invalidate
    // this test.
    if *a != 5 {
        c_take5(a);
    } else {
        std::alloc::dealloc(a, layout);
    }
    5
}
