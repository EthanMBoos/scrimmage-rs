ARG REFERENCE_IMAGE=scrimmage-rs-reference:latest
FROM rust:1.98-slim-bookworm AS rust-build
WORKDIR /source
COPY Cargo.toml Cargo.lock ./
COPY crates ./crates
COPY reference/benchmark_runner.rs ./reference/benchmark_runner.rs
RUN cargo bench --locked -p scrimmage-core --bench reference --no-run \
    && find target/release/deps -maxdepth 1 -type f -executable -name 'reference-*' \
       -exec cp {} /usr/local/bin/scrimmage-benchmark \;
RUN rustc --version --verbose > /rust-compiler.txt

FROM ${REFERENCE_IMAGE}
COPY --from=rust-build /usr/local/bin/scrimmage-benchmark /usr/local/bin/scrimmage-benchmark
COPY --from=rust-build /rust-compiler.txt /rust-compiler.txt
ENTRYPOINT ["python3", "/rust/reference/benchmark.py", "--inside"]
