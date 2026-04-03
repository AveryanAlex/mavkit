use super::UpdateContext;
use crate::dialect;

pub(super) fn handle_status_text(context: &UpdateContext<'_>, data: &dialect::STATUSTEXT_DATA) {
    if let Some(status_text) = context
        .writers
        .telemetry_metrics
        .message_writers
        .status_text
        .ingest(context.header, data, None)
    {
        let _ = context
            .writers
            .statustext
            .send(Some(crate::state::StatusMessage {
                text: status_text.text,
                severity: crate::state::MavSeverity::from_mav(status_text.severity),
            }));
    }
}
