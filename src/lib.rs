//! A heap allocator for PIC32 controllers
//!
//! # Example
//!
//! ```
//! #![feature(global_allocator)]
//! #![feature(lang_items)]
//!
//! // Plug in the allocator crate
//! extern crate alloc;
//! use mips_rt;
//!
//! use alloc::Vec;
//! use alloc_pic32::Pic32Heap;
//!
//! #[global_allocator]
//! static ALLOCATOR: Pic32Heap = Pic32Heap::empty();
//!
//! entry!(main);
//!
//! fn main() -> ! {
//!     // Initialize the allocator BEFORE you use it
//!     let start = mips_rt::heap_start() as usize;
//!     let size = 1024; // in bytes
//!     unsafe { ALLOCATOR.init(start, size) }
//!
//!     let mut xs = Vec::new();
//!     xs.push(1);
//!
//!     loop { /* .. */ }
//! }
//!
//! // required: define how Out Of Memory (OOM) conditions should be handled
//! // *if* no other crate has already defined `oom`
//! #[lang = "oom"]
//! #[no_mangle]
//! pub fn rust_oom() -> ! {
//!     // ..
//! }
//!
//!
//! // omitted: exception handlers
//! ```

#![feature(allocator_api)]
#![no_std]

extern crate alloc;
extern crate mips_mcu;
extern crate linked_list_allocator;

use core::alloc::{GlobalAlloc, Layout};
use core::cell::UnsafeCell;
use core::ptr::NonNull;

use mips_mcu::interrupt;
use linked_list_allocator::Heap;

pub struct Pic32Heap {
    heap: UnsafeCell<Heap>,
}

impl Pic32Heap {
    /// Crate a new UNINITIALIZED heap allocator
    ///
    /// You must initialize this heap using the
    /// [`init`](struct.Pic32Heap.html#method.init) method before using the allocator.
    pub const fn empty() -> Pic32Heap {
        Pic32Heap {
            heap: UnsafeCell::new(Heap::empty()),
        }
    }

    /// Wrapper for working with the heap
    ///
    /// This wrapper disables the Interrupts and provides a mutable reference to
    /// the inner Heap structure.
    fn with_heap<F,R>(&self, f: F) -> R
    where
        F: FnOnce(&mut Heap) -> R,
    {
        interrupt::free(|_| {
            unsafe { f(&mut *self.heap.get()) }
        })
    }

    /// Initializes the heap
    ///
    /// This function must be called BEFORE you run any code that makes use of the
    /// allocator.
    ///
    /// `start_addr` is the address where the heap will be located.
    ///
    /// `size` is the size of the heap in bytes.
    ///
    /// Note that:
    ///
    /// - The heap grows "upwards", towards larger addresses. Thus `end_addr` must
    ///   be larger than `start_addr`
    ///
    /// - The size of the heap is `(end_addr as usize) - (start_addr as usize)`. The
    ///   allocator won't use the byte at `end_addr`.
    ///
    /// # Unsafety
    ///
    /// Obey these or Bad Stuff will happen.
    ///
    /// - This function must be called exactly ONCE.
    /// - `size > 0`
    pub unsafe fn init(&self, start_addr: usize, size: usize) {
        self.with_heap(|heap|{
            heap.init(start_addr, size);
        });
    }
}

unsafe impl GlobalAlloc for Pic32Heap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.with_heap(|heap|{
            heap.allocate_first_fit(layout)
                .ok()
                .map_or(0 as *mut u8, |allocation| allocation.as_ptr())
        })
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        self.with_heap(|heap|{
            heap.deallocate(NonNull::new_unchecked(ptr), layout);
        });
    }
}

unsafe impl Sync for Pic32Heap {}
