
/** @type {import("markdownlint").Rule} */
module.exports = {
  names: ["warn-stale-review"],
  description: "Warn when a document has not been reviewed recently.",
  information: new URL(
    "https://unl-lunabotics.github.io/docs/Administrative/The-Linter.html#custom-rule-required-last-reviewed-tag"
  ),
  tags: ["credit"],
  parser: "none",
  function: (params, onError) => {
    const path = require("path");
    if (path.basename(params.name) === "index.md") return;

    const { lines, config = {} } = params;

    // Default to 182 days if not configured.
    const configuredMaxAgeDays = config.maxAgeDays;
    const maxAgeDays =
      Number.isInteger(configuredMaxAgeDays) &&
      configuredMaxAgeDays >= 0
        ? configuredMaxAgeDays
        : 182;

    const lastReviewedPrefixRx = /^>\s*Last Reviewed:/;
    const lastReviewedRx =
      /^>\s*Last Reviewed:\s*(\d{4})-(\d{2})-(\d{2})$/;

    // Find the end of the document.
    let idx = lines.length - 1;
    while (idx >= 0 && !lines[idx].trim()) {
      idx--;
    }

    // Collect the final contiguous blockquote footer.
    const footer = [];
    for (
      let i = idx;
      i >= 0 && lines[i].trim().startsWith(">");
      i--
    ) {
      footer.unshift(i);
    }

    // Find the last-reviewed tag, even if its date is malformed.
    const reviewIndex = footer.find(i =>
      lastReviewedPrefixRx.test(lines[i].trim())
    );

    // Missing or malformed tags are handled by the required-tag rule.
    if (reviewIndex === undefined) return;

    const reviewLine = lines[reviewIndex].trim();
    const match = reviewLine.match(lastReviewedRx);

    if (!match) return;

    const year = Number(match[1]);
    const month = Number(match[2]);
    const day = Number(match[3]);

    // Parse a date-only value and reject impossible calendar dates.
    const reviewedDate = new Date(0);
    reviewedDate.setUTCHours(0, 0, 0, 0);
    reviewedDate.setUTCFullYear(year, month - 1, day);

    if (
      reviewedDate.getUTCFullYear() !== year ||
      reviewedDate.getUTCMonth() !== month - 1 ||
      reviewedDate.getUTCDate() !== day
    ) {
      return;
    }

    // Compare calendar dates, not elapsed hours.
    const now = new Date();
    const today = new Date(0);
    today.setUTCHours(0, 0, 0, 0);
    today.setUTCFullYear(
      now.getFullYear(),
      now.getMonth(),
      now.getDate()
    );

    const millisecondsPerDay = 24 * 60 * 60 * 1000;
    const ageDays = Math.floor(
      (today.getTime() - reviewedDate.getTime()) /
        millisecondsPerDay
    );

    // Only warn when the date is strictly older than the threshold.
    if (ageDays <= maxAgeDays) return;

    onError({
      lineNumber: reviewIndex + 1,
      detail:
        `Review recommended: last reviewed ${ageDays} days ago ` +
        `(threshold: ${maxAgeDays} days).`,
      context: lines[reviewIndex]
    });
  }
};
