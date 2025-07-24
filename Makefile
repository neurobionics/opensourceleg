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

.PHONY: build
build: clean-build ## Build wheel file using uv
	@echo "🚀 Creating wheel file"
	@uv build

.PHONY: clean-build
clean-build: ## clean build artifacts
	@rm -rf dist

.PHONY: publish
publish: ## publish a release to pypi.
	@echo "🚀 Publishing to PyPI."
	@uv publish --token $(PYPI_TOKEN)

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
