use camlib::{operation::OperationPlan, stock::Stock, stream::StreamManifest};

fn main() -> Result<(), String> {
    let job: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/helical-pocket.json"))
            .map_err(|e| e.to_string())?;
    job.validate()?;
    let mut stock = Stock::new(job.stock.height().unwrap().clone())?;
    let mut position = job.initial_position_mm;
    let mut next_id = 1;
    for operation in job.operations {
        let tool = job
            .tools
            .iter()
            .find(|t| t.id == operation.tool())
            .ok_or("missing tool")?;
        let prepared = tool.prepare()?;
        let mut plan =
            OperationPlan::prepare_with_stock(operation, tool, &stock, position, next_id, &|| {
                false
            })?;
        while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false)? {
            let delta = stock.preview_sweep(&prepared, &motion, 1., job.sweep, &|| false)?;
            stock.apply_delta(&delta)?;
            position = motion.curve.end();
            next_id = motion.id + 1;
            println!(
                "{}",
                serde_json::to_string(&motion).map_err(|e| e.to_string())?
            );
        }
        plan.finish_report(&|| false)?;
    }
    eprintln!(
        "{} motions; removed stock: {:?}",
        next_id - 1,
        stock.volume()
    );
    Ok(())
}
