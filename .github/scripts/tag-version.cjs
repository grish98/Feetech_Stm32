// GitHub tags are the only project version source. CI serializes main updates.
module.exports = async function tagVersion({ github, context, core }) {
  const repo = context.repo;
  const tags = await github.paginate(github.rest.repos.listTags, {
    ...repo,
    per_page: 100,
  });
  const versions = tags
    .filter(tag => /^v0\.[1-9]\d*$/.test(tag.name));

  // Re-running CI for a tagged commit must not allocate another version.
  const existing = versions.find(tag => tag.commit.sha === context.sha);
  if (existing) {
    core.info(`${context.sha} already has version ${existing.name}`);
    return;
  }

  // An old workflow rerun must not publish an older commit as the latest version.
  const { data: main } = await github.rest.git.getRef({ ...repo, ref: 'heads/main' });
  if (main.object.sha !== context.sha) {
    core.info('Skipping superseded main commit. Only the current main tip is tagged.');
    return;
  }

  const latest = versions.reduce((max, tag) => {
    const minor = BigInt(tag.name.slice(3));
    return minor > max ? minor : max;
  }, 0n);
  const version = `v0.${latest + 1n}`;
  await github.rest.git.createRef({
    ...repo,
    ref: `refs/tags/${version}`,
    sha: context.sha,
  });
  core.info(`Created ${version} at ${context.sha}`);
};
