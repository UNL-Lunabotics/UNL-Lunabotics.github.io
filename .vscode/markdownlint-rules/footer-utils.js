
const TAG_PREFIXES = {
  author: /^>\s*Author:/,
  "last-reviewed": /^>\s*Last Reviewed:/
};

function getFooterInfo(params) {
  const { lines } = params;

  let end = lines.length - 1;
  while (end >= 0 && !lines[end].trim()) end--;

  // Find the contiguous blockquote footer.
  let start = end;
  while (start >= 0 && lines[start].trim().startsWith(">")) {
    start--;
  }

  const footerIndices = [];
  for (let i = start + 1; i <= end; i++) {
    footerIndices.push(i);
  }

  const findTag = type =>
    footerIndices.find(i =>
      TAG_PREFIXES[type].test(lines[i].trim())
    );

  const authorTagIndex = findTag("author");
  const lastReviewedTagIndex = findTag("last-reviewed");

  return {
    end,
    footerIndices,
    authorTagIndex,
    lastReviewedTagIndex,
    hasAnyTag:
      authorTagIndex !== undefined ||
      lastReviewedTagIndex !== undefined
  };
}

// Append text at the end, accounting for existing trailing blank lines.
function appendAtEnd(params, text, blankLineBefore = false) {
  const { lines } = params;
  const { end } = getFooterInfo(params);

  if (end < 0) {
    return {
      lineNumber: 1,
      editColumn: 1,
      deleteCount: lines[0]?.length ?? 0,
      insertText: text
    };
  }

  if (end + 1 < lines.length) {
    // Insert into the first existing trailing blank line.
    return {
      lineNumber: end + 2,
      editColumn: 1,
      deleteCount: 0,
      insertText: `${blankLineBefore ? "\n" : ""}${text}`
    };
  }

  return {
    lineNumber: end + 1,
    editColumn: lines[end].length + 1,
    deleteCount: 0,
    insertText: `\n${blankLineBefore ? "\n" : ""}${text}`
  };
}

function createFooterTagFix(params, type, template) {
  const { lines } = params;
  const info = getFooterInfo(params);

  if (
    type === "author" &&
    info.lastReviewedTagIndex !== undefined
  ) {
    const reviewIndex = info.lastReviewedTagIndex;

    // Reuse an existing spacer line when possible.
    const spacerIndex =
      reviewIndex > 0 &&
      lines[reviewIndex - 1].trim() === ">"
        ? reviewIndex - 1
        : undefined;

    if (spacerIndex !== undefined) {
      return {
        lineNumber: spacerIndex + 1,
        editColumn: 1,
        deleteCount: 0,
        insertText: `${template}\n`
      };
    }

    return {
      lineNumber: reviewIndex + 1,
      editColumn: 1,
      deleteCount: 0,
      insertText: `${template}\n>\n`
    };
  }

  if (type === "last-reviewed") {
    const spacerAlreadyExists =
      info.end >= 0 && lines[info.end].trim() === ">";

    const text = spacerAlreadyExists
      ? template
      : `>\n${template}`;

    return appendAtEnd(params, text);
  }

  return appendAtEnd(params, template);
}

function createCompleteFooterFix(params, authorTemplate, reviewTemplate) {
  return appendAtEnd(
    params,
    `${authorTemplate}\n>\n${reviewTemplate}`,
    true
  );
}

function replaceLineFix(lineIndex, template) {
  return {
    lineNumber: lineIndex + 1,
    editColumn: 1,
    deleteCount: -1,
    insertText: template
  };
}

module.exports = {
  getFooterInfo,
  createFooterTagFix,
  createCompleteFooterFix,
  replaceLineFix
};
