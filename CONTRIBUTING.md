# Contributing to JARGON

First off, thank you for considering contributing to JARGON and sharing your ideas and efforts. We appreciate all
contributions great and small.

These guidelines are intended to promote good discussions and streamline the improvement and development process. This
contribution guide is still a work in progress and will likely see revisions in the future.

The community shall graciously accept all contributions, from bug reports, documentation fixes and additions, new features or feature requests, tutorials or guidelines, etc.

# Code of conduct
We use the [Contributor Covenant](CODE OF CONDUCT.md) code of conduct. 
In general, please keep a warm and productive atmosphere in discussions, be kind, and use common sense.

# Submitting issues
If you have a:
- Bug report
- Documentation request
- Feature request
- Substantial question

Please submit a github issue. Be sure to check that an issue with your content already exists; upvoting
or commenting on issues helps bring them more attention.

Good issues have:
- A descriptive title
- For bug reports a detailed enough explanation of the bug to reproduce it. (We want to know what the bug is!)
- For feature requests, although not all required, consider having the following:
  - What the feature is
  - Why the feature is desired
  - How the feature might be implemented

Issues will be checked and responded to at least weekly.

# Contributing

Contributions are done through _Pull Requests_.

>Working on your first Pull Request? You can learn how here: [How to Contribute to an Open Source Project on GitHub](https://egghead.io/series/how-to-contribute-to-an-open-source-project-on-github).

Development is done in branches prefixed with `develop`, or simply the `develop` branch.

For "small" fixes (fixes that only change a few lines or do not significantly affect code structure), a
pull request can be submitted directly.

For larger fixes or improvements (anything with breaking changes, significantly restructures existing code,
or adds new features), a corresponding issue should be opened first.
We will most likely approve the development of most feature requests at this point.

You can see existing issues for ideas on what contributions are right now wanted.

Pull requests will be checked and responded to weekly.

For coding specifically:
- It is recommend to use either Intellij IDEA or Android Studio. That way, existing project formatting and 
  inspections can be integrated.
- Contributions both in Java and Kotlin will be accepted, although Java code will be converted to Kotlin
 (using J2K + manual edits)
    - Trust me; Kotlin is a language worth learning and is better than Java so go learn it anyways
- We strongly suggest including a unit test for new features.
- If a pull request will be reviewed and possibly edited before integrating it with the main body of JARGON
- We may rebase commits if your git history is convoluted (basically cleaning up the git history)
