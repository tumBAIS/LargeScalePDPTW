#![feature(test)]
extern crate test;

extern crate fp_decimal_type;

#[cfg(test)]
mod benches {
    use rand::{Rng, SeedableRng};
    use rand_pcg::Pcg64;
    use test::{black_box, Bencher};
    use fp_decimal_type::*;

    fp_decimal_type::define_fpd_type!(name: NumI64P3, type: i64, precision: 3);

    #[bench]
    fn add(b: &mut Bencher) {
        let k = 100_000_usize;
        let r = -100_i64..100_i64;
        let mut rand = Pcg64::seed_from_u64(842);
        let set1: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        let set2: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        b.iter(|| {
            for i in 0..k {
                black_box(set1[i] + set2[i]);
            }
        });
    }

    #[bench]
    fn sub(b: &mut Bencher) {
        let k = 100_000_usize;
        let r = -100_i64..100_i64;
        let mut rand = Pcg64::seed_from_u64(842);
        let set1: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        let set2: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        b.iter(|| {
            for i in 0..k {
                black_box(set1[i] - set2[i]);
            }
        });
    }

    #[bench]
    fn mul(b: &mut Bencher) {
        let k = 100_000_usize;
        let r = -100_i64..100_i64;
        let mut rand = Pcg64::seed_from_u64(842);
        let set1: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        let set2: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        b.iter(|| {
            for i in 0..k {
                black_box(set1[i] * set2[i]);
            }
        });
    }

    #[bench]
    fn div(b: &mut Bencher) {
        let k = 100_000_usize;
        let r = -100_i64..100_i64;
        let mut rand = Pcg64::seed_from_u64(842);
        let set1: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        let set2: Vec<NumI64P3> = (0..k).map(|_| rand.gen_range(r.clone()).into()).collect();
        b.iter(|| {
            for i in 0..k {
                black_box(set1[i] / set2[i]);
            }
        });
    }
}