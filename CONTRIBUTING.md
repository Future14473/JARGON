# Contributing to JARGON

Thank you for considering contributing to JARGON and sharing your ideas and efforts. We appreciate all
contributions

These guidelines are intended to promote productive discourse and streamline the improvement and development process.

Note this contribution guide is still a work in progress and will likely see revisions in the future.


# Code of conduct
We use the [Contributor Covenant](CODE OF CONDUCT.md) code of conduct. 
In general, be kind, keep a warm and productive atmospheres,, and use common sense.

# Submitting issues
If you have a:
- Bug report
- Documentation/tutorial request
- Feature request
- Substantial question

Please submit a github issue. Be sure to check if a similar issue exists to avoid duplicates; prefer upvoting or commenting on previous issues.

Good issues have:
- A descriptive title
- For bug reports a detailed enough explanation of the bug to reproduce it.
- For feature requests, although not all required, consider having the following:
  - What the feature is
  - Why the feature is desired
  - How the feature might be implemented

# Contributing

Contributions are done through _Pull Requests_.

>Working on your first Pull Request? You can learn how here: [How to Contribute to an Open Source Project on GitHub](https://egghead.io/series/how-to-contribute-to-an-open-source-project-on-github).

Development is done in branches prefixed with `develop`, or simply the `develop` branch.

For "small" fixes (fixes that only change a few lines or do not significantly affect code structure), a
pull request can be submitted directly.

For larger changes (anything with breaking changes, significantly restructures existing code,
or adds new features), a corresponding issue should be opened first.
We will most likely approve the development of most feature requests at this point.

You can see existing issues for ideas on contributions.


## Code guidelines
- It is recommend to use Intellij IDEA/Android Studio. That way, existing project formatting and inspections can be integrated.
- Contributions both in Java and Kotlin will be accepted, although Java code will be converted to Kotlin
 (using J2K + manual edits)
- We strongly suggest including unit tests for new features
