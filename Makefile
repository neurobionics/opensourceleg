.PHONY: install
install: ## Install the uv environment and install the pre-commit hooks
	@echo "🚀 Creating virtual environment and installing dependencies using uv"
	@uv sync
	@uv run pre-commit install

.PHONY: check
check: ## Run code quality tools.
	@echo "🚀 Checking uv lock file consistency: Running uv sync --check"
	@uv sync --check
	@echo "🚀 Linting code: Running pre-commit"
	@uv run pre-commit run -a
	@echo "🚀 Static type checking: Running mypy"
	@uv run mypy
	@echo "🚀 Checking for obsolete dependencies: Running deptry"
	@uv run deptry . --ignore DEP002,DEP001,DEP003

.PHONY: test
test: ## Test the code with pytest
	@echo "🚀 Testing code: Running pytest"
	@uv run pytest --cov --cov-config=pyproject.toml --cov-report=xml

.PHONY: build-rust
build-rust: ## Build the Rust extension in development mode
	@echo "🚀 Building Rust extension"
	@uv run maturin develop

.PHONY: build-rust-release
build-rust-release: ## Build the Rust extension in release mode
	@echo "🚀 Building Rust extension (release mode)"
	@uv run maturin develop --release

.PHONY: build-rust-arm64
build-rust-arm64: ## Cross-compile Rust extension for ARM64 (Raspberry Pi 4/5)
	@echo "🚀 Cross-compiling Rust extension for ARM64"
	@uv run maturin build --release --target aarch64-unknown-linux-gnu

.PHONY: build-rust-armhf
build-rust-armhf: ## Cross-compile Rust extension for ARMhf (Raspberry Pi 3/older)
	@echo "🚀 Cross-compiling Rust extension for ARMhf"
	@uv run maturin build --release --target armv7-unknown-linux-gnueabihf

.PHONY: build-rust-all
build-rust-all: ## Build Rust extension for all target architectures
	@echo "🚀 Building Rust extension for all targets"
	@uv run maturin build --release
	@make build-rust-arm64
	@make build-rust-armhf

.PHONY: build-rust-docker
build-rust-docker: ## Cross-compile using Docker (works on all platforms)
	@echo "🚀 Cross-compiling Rust extension using Docker"
	@docker build -f Dockerfile.cross -t opensourceleg-cross .
	@docker run --rm -v $(PWD)/dist:/workspace/dist opensourceleg-cross

.PHONY: setup-cross
setup-cross: ## Setup cross-compilation environment (Linux only)
	@echo "🚀 Setting up cross-compilation environment"
	@./scripts/setup-cross-compilation.sh

.PHONY: build
build: clean-build build-rust-release ## Build wheel file using uv
	@echo "🚀 Creating wheel file"
	@uv build

.PHONY: clean-build
clean-build: ## clean build artifacts
	@rm -rf dist target

.PHONY: clean-rust
clean-rust: ## Clean Rust build artifacts
	@echo "🚀 Cleaning Rust build artifacts"
	@cargo clean

.PHONY: publish
publish: ## publish a release to pypi.
	@echo "🚀 Publishing: Dry run."
	@uv publish --dry-run --token $(PYPI_TOKEN)
	@echo "🚀 Publishing."
	@uv publish --token $(PYPI_TOKEN)

.PHONY: build-and-publish
build-and-publish: build publish ## Build and publish.

.PHONY: docs-test
docs-test: ## Test if documentation can be built without warnings or errors
	@PYTHONWARNINGS="ignore::DeprecationWarning" uv run mkdocs build -s

.PHONY: docs
docs: ## Build and serve the documentation
	@PYTHONWARNINGS="ignore::DeprecationWarning" uv run mkdocs serve

.PHONY: docs-deploy
docs-deploy: ## Deploy the documentation to GitHub pages
	@PYTHONWARNINGS="ignore::DeprecationWarning" uv run mkdocs gh-deploy --force

.PHONY: help
help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
