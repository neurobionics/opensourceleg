# How to contribute

## Installing dev dependencies

To get started you would need to install [`python3.9`](https://www.python.org/downloads/) and [`poetry`](https://python-poetry.org/docs/#installing-with-the-official-installer). Please follow the instructions on their official websites.

* We use *poetry* to manage our python [dependencies](https://github.com/python-poetry/poetry). Please make sure that *poetry* is added to your **PATH** variable after installation and you can run `poetry` command in your terminal or command prompt.

* After cloning the *opensourceleg* repository, please activate your *virtualenv* by running `poetry shell` command. This will create an isolated virtual environment for development. 

* To install dependencies and prepare [`pre-commit`](https://pre-commit.com/) hooks you would need to run these commands from the root of the repository:

    ```bash
    make install
    make pre-commit-install
    ```

## Submitting your code

Many checks are configured for this project. 
* Command `make check-codestyle` will check black, isort and darglint.
* Command `make lint` will check types, docstrings and security using [Mypy](https://pypi.org/project/mypy/), [Darglint](https://pypi.org/project/darglint/), [Pydocstyle](https://pypi.org/project/pydocstyle/)
* Command `make check-safety` will look at the security of your code and dependencies using [Safety](https://pypi.org/project/safety/) and [Bandit](https://pypi.org/project/bandit/).

Before submitting your code please do the following steps:

1. Add any changes you want
1. Add tests for the new changes
1. Edit documentation if you have changed something significant
1. Run `make codestyle` to format your changes.
1. Run `make lint` to ensure that types, security and docstrings are okay.
1. Run `make check-safety` to ensure that your code is secure.

Your code will be checked by our CI/CD pipeline once you submit a pull request. Happy coding! 

## Other ways you can help

You can contribute by spreading a word about this library.
It would also be a huge contribution to write
a short article on how you are using this project.
You can also share your best practices with us.
